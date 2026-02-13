#!/usr/bin/env python3
"""
Step Response Automation Framework
Automated testing with parametrized speed/force configurations
"""

import serial
import struct
import time
import sys
import select
import logging
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Callable
from pathlib import Path
from datetime import datetime

import numpy as np
import polars as pl
import matplotlib.pyplot as plt

# ============================================================================
# TEST CONFIGURATION
# ============================================================================

@dataclass
class TestPoint:
    """Single test configuration point."""
    speed: int                  # Motor speed command
    duration_ms: int            # Step duration
    max_force_N: float          # Force limit
    description: str = ""       # Optional label
    
    def __str__(self) -> str:
        desc = f" ({self.description})" if self.description else ""
        return f"Speed={self.speed:4d} | Duration={self.duration_ms:4d}ms | MaxForce={self.max_force_N:6.1f}N{desc}"

@dataclass
class TestSuite:
    """Collection of test points to execute sequentially."""
    name: str
    test_points: List[TestPoint]
    
    def __str__(self) -> str:
        return f"TestSuite: {self.name} ({len(self.test_points)} tests)"

# ============================================================================
# SERIAL CONNECTION (simplified from main system_identification.py)
# ============================================================================

class SerialConnection:
    """Manages serial communication with ESP32."""
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 460800):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.logger = logging.getLogger(__name__)
    
    def connect(self) -> bool:
        """Establish serial connection."""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.001)
            self.logger.info(f"Connected to {self.port} @ {self.baudrate} baud")
            time.sleep(0.1)
            self.serial.reset_input_buffer()
            return True
        except serial.SerialException as e:
            self.logger.error(f"Failed to connect: {e}")
            return False
    
    def disconnect(self) -> None:
        """Close serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.logger.info("Serial connection closed")
    
    def clear_input_buffer(self) -> None:
        """Flush serial input buffer."""
        if self.serial and self.serial.is_open:
            self.serial.reset_input_buffer()
    
    def write_command(self, cmd: str) -> None:
        """Send ASCII command to ESP32."""
        if not self.serial:
            raise RuntimeError("Serial not connected")
        full_cmd = cmd if cmd.endswith('\n') else cmd + '\n'
        self.serial.write(full_cmd.encode('ascii'))
    
    def read_exact(self, size: int) -> Optional[bytes]:
        """Read exactly N bytes from serial buffer."""
        if not self.serial:
            return None
        
        buf = b''
        while len(buf) < size:
            chunk = self.serial.read(size - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf
    
    def is_data_ready(self) -> bool:
        """Check if serial data is available."""
        if not self.serial:
            return False
        return self.serial.in_waiting > 0
    
    def sync_to_header(self, status_header: bytes, step_end_header: bytes) -> Optional[str]:
        """Synchronize to binary protocol header."""
        if not self.serial:
            return None
        sync_buf = b''
        timeout_start = time.time()
        
        while time.time() - timeout_start < 0.1:
            byte = self.serial.read(1)
            if not byte:
                continue
            
            sync_buf += byte
            if len(sync_buf) > 2:
                sync_buf = sync_buf[-2:]
            
            if len(sync_buf) == 2:
                if sync_buf == step_end_header:
                    return 'step_end'
                elif sync_buf == status_header:
                    return 'status'
        
        return None

# ============================================================================
# DATA COLLECTION
# ============================================================================

@dataclass
class SensorData:
    """Single sensor reading."""
    timestamp_s: float
    timestamp_us: int
    force_N: float
    cable_length_mm: float

class DataBuffer:
    """Manages sensor data for a single test."""
    
    def __init__(self):
        self.data: List[SensorData] = []
        self.logger = logging.getLogger(__name__)
    
    def append(self, sample: SensorData) -> None:
        """Add sensor reading."""
        self.data.append(sample)
    
    def clear(self) -> None:
        """Reset buffer."""
        self.data.clear()
    
    def __len__(self) -> int:
        return len(self.data)
    
    def to_polars_df(self) -> pl.DataFrame:
        """Convert to Polars DataFrame."""
        if not self.data:
            raise ValueError("Buffer is empty")
        
        return pl.DataFrame({
            "timestamp_s": [d.timestamp_s for d in self.data],
            "timestamp_us": [d.timestamp_us for d in self.data],
            "force_N": [d.force_N for d in self.data],
            "cable_length_mm": [d.cable_length_mm for d in self.data]
        })
    
    def save_parquet(self, filepath: Path) -> None:
        """Save data as Parquet."""
        df = self.to_polars_df()
        df.write_parquet(filepath)
        self.logger.info(f"Saved {len(df)} samples to {filepath}")
    
    def get_sample_rate(self) -> float:
        """Calculate effective sample rate."""
        if len(self.data) < 2:
            return 0.0
        duration = self.data[-1].timestamp_s - self.data[0].timestamp_s
        return len(self.data) / duration if duration > 0 else 0.0

# ============================================================================
# TEST RUNNER (CORE AUTOMATION)
# ============================================================================

class StepResponseAutomation:
    """Automated step response test executor."""
    
    def __init__(self, serial_port: str = '/dev/ttyUSB0', output_dir: Optional[Path] = None):
        self.serial = SerialConnection(serial_port)
        self.output_dir = output_dir or Path.cwd() / "test_results"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.buffer = DataBuffer()
        self.logger = logging.getLogger(__name__)
        
        # Protocol config
        self.status_header = b'\xaa\x55'
        self.step_end_header = b'\xbb\x66'
        self.struct_format = '<I f f'
        self.struct_size = 12
    
    def connect(self) -> bool:
        """Establish serial connection."""
        return self.serial.connect()
    
    def disconnect(self) -> None:
        """Close serial connection."""
        self.serial.disconnect()
    
    def wait_for_step_end(self, timeout_s: float = 5.0) -> bool:
        """Wait for step-end header after command."""
        start_time = time.time()
        
        while time.time() - start_time < timeout_s:
            if not self.serial.is_data_ready():
                time.sleep(0.001)
                continue
            
            header_type = self.serial.sync_to_header(self.status_header, self.step_end_header)
            
            if header_type == 'status':
                # Read status packet (ignore during wait)
                data = self.serial.read_exact(self.struct_size)
                if data:
                    try:
                        timestamp_us, force_N, cable_length_mm = struct.unpack(self.struct_format, data)
                        sample = SensorData(
                            timestamp_s=time.time(),
                            timestamp_us=timestamp_us,
                            force_N=force_N,
                            cable_length_mm=cable_length_mm
                        )
                        self.buffer.append(sample)
                    except struct.error:
                        pass
            
            elif header_type == 'step_end':
                self.logger.warning("🎯 STEP END received")
                return True
        
        self.logger.warning(f"⏱️ Timeout waiting for step end ({timeout_s}s)")
        return False
    
    def execute_test_point(self, test_point: TestPoint) -> bool:
        """Execute single test point."""
        self.logger.info(f"▶️ Starting test: {test_point}")
        
        # Set max force
        self.serial.write_command(f"f {test_point.max_force_N}")
        time.sleep(0.5)
        
        # Clear buffer for this test
        self.buffer.clear()
        self.serial.clear_input_buffer()
        
        # Send step command
        self.serial.write_command(f"step {test_point.speed} {test_point.duration_ms}")
        self.logger.info(f"Sent: step {test_point.speed} {test_point.duration_ms}")
        
        # Wait for step to complete
        success = self.wait_for_step_end(timeout_s=test_point.duration_ms / 1000.0 + 3.0)
        
        if success:
            sample_rate = self.buffer.get_sample_rate()
            self.logger.info(f"   ✅ Completed | {len(self.buffer)} samples @ {sample_rate:.0f}Hz")
        else:
            self.logger.warning(f"   ❌ Failed or timeout")
        
        return success
    
    def reset_to_zero(self) -> bool:
        """Reset rope length to 0 using 'r' command. Monitors until cable_length_mm < 1mm."""
        self.logger.info("↩️ Resetting to rope length 0...")
        
        # Clear buffer first
        self.serial.clear_input_buffer()
        time.sleep(0.1)
        
        # Send reset command
        self.serial.write_command("r")
        
        # Wait for reset to complete, monitoring cable length
        timeout_s = 30.0  # Increased timeout for long cable lengths
        start_time = time.time()
        last_cable_length = None
        samples_checked = 0
        
        while time.time() - start_time < timeout_s:
            if not self.serial.is_data_ready():
                time.sleep(0.01)
                continue
            
            header_type = self.serial.sync_to_header(self.status_header, self.step_end_header)
            
            if header_type == 'status':
                data = self.serial.read_exact(self.struct_size)
                if data:
                    try:
                        timestamp_us, force_N, cable_length_mm = struct.unpack(self.struct_format, data)
                        last_cable_length = cable_length_mm
                        samples_checked += 1
                        
                        # Log progress every 50 samples (~100ms at 600Hz)
                        if samples_checked % 50 == 0:
                            self.logger.info(f"   Resetting... Cable length: {cable_length_mm:.3f}mm")
                        
                        # Check if reset is complete
                        if abs(cable_length_mm) < 1.0:
                            self.logger.info(f"✅ Reset complete | Cable length: {cable_length_mm:.3f}mm | Samples: {samples_checked}")
                            return True
                    except struct.error:
                        pass
        
        # Timeout fallback
        cable_str = f"{last_cable_length:.3f}mm" if last_cable_length is not None else "unknown"
        self.logger.warning(f"⏱️ Reset timeout ({timeout_s}s) | Last cable length: {cable_str} | Samples: {samples_checked}")
        return False
    
    def save_test_result(self, test_index: int, test_point: TestPoint) -> Path:
        """Save test data to Parquet file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"test_{test_index:02d}_{test_point.speed}spd_{test_point.max_force_N:.1f}N_{timestamp}.parquet"
        filepath = self.output_dir / filename
        
        self.buffer.save_parquet(filepath)
        return filepath
    
    def generate_plots(self, test_index: int, test_point: TestPoint, timestamp: str) -> Path:
        """Generate 3-panel plot (Force, Cable Length, Cable Velocity) and save as PNG."""
        if len(self.buffer) < 2:
            self.logger.warning("Cannot generate plots: insufficient data points")
            return None
        
        try:
            df = self.buffer.to_polars_df()
            time_s = df["timestamp_s"].to_numpy() - df["timestamp_s"][0]
            
            fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
            
            # Force plot
            axes[0].plot(time_s, df["force_N"].to_numpy(), 'b-', linewidth=0.8)
            axes[0].set_ylabel("Force [N]", fontsize=11)
            axes[0].grid(True, alpha=0.3)
            axes[0].set_title(f"Step Response Test #{test_index} | Speed={test_point.speed} | Duration={test_point.duration_ms}ms | MaxForce={test_point.max_force_N}N", 
                             fontsize=12, fontweight='bold')
            
            # Cable length plot
            axes[1].plot(time_s, df["cable_length_mm"].to_numpy(), 'r-', linewidth=0.8)
            axes[1].set_ylabel("Cable Length [mm]", fontsize=11)
            axes[1].grid(True, alpha=0.3)
            
            # Cable velocity plot (numerical gradient)
            cable_velocity = np.gradient(df["cable_length_mm"].to_numpy()) / np.gradient(time_s)
            axes[2].plot(time_s, cable_velocity, 'g-', linewidth=0.8)
            axes[2].set_ylabel("Cable Velocity [mm/s]", fontsize=11)
            axes[2].set_xlabel("Time [s]", fontsize=11)
            axes[2].grid(True, alpha=0.3)
            
            plt.tight_layout()
            
            # Save plot
            plot_filename = f"test_{test_index:02d}_{test_point.speed}spd_{test_point.max_force_N:.1f}N_{timestamp}.png"
            plot_path = self.output_dir / plot_filename
            plt.savefig(plot_path, dpi=200, bbox_inches='tight')
            plt.close()  # Close figure to free memory
            
            self.logger.info(f"Plot saved: {plot_path}")
            return plot_path
        
        except Exception as e:
            self.logger.error(f"Failed to generate plots: {e}")
            return None
    
    def run_test_suite(self, suite: TestSuite, on_test_complete: Optional[Callable] = None) -> bool:
        """Execute entire test suite."""
        self.logger.info(f"\n{'='*70}")
        self.logger.info(f"STARTING {suite}")
        self.logger.info(f"{'='*70}\n")
        
        results = []
        
        for idx, test_point in enumerate(suite.test_points, 1):
            self.logger.info(f"\n[{idx}/{len(suite.test_points)}] Test Point")
            
            # Execute test
            success = self.execute_test_point(test_point)
            
            # Save result
            if success:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filepath = self.save_test_result(idx, test_point)
                
                # Generate plot for successful tests
                plot_path = self.generate_plots(idx, test_point, timestamp)
                
                results.append({
                    'index': idx,
                    'test_point': test_point,
                    'samples': len(self.buffer),
                    'filepath': filepath,
                    'plot_path': plot_path,
                    'status': 'success'
                })
            else:
                results.append({
                    'index': idx,
                    'test_point': test_point,
                    'samples': 0,
                    'filepath': None,
                    'plot_path': None,
                    'status': 'failed'
                })
            
            # Execute custom callback (e.g., for plotting)
            if on_test_complete:
                on_test_complete(idx, test_point, self.buffer, success)
            
            # Reset for next test (except last)
            if idx <= len(suite.test_points):
                self.reset_to_zero()
                time.sleep(2.0)  # Settle time before next test
                self.serial.clear_input_buffer()
        
        # Summary
        self.logger.info(f"\n{'='*70}")
        self.logger.info(f"TEST SUITE SUMMARY: {suite.name}")
        self.logger.info(f"{'='*70}")
        
        successful = sum(1 for r in results if r['status'] == 'success')
        self.logger.info(f"Results: {successful}/{len(suite.test_points)} successful")
        
        for result in results:
            status_emoji = "✅" if result['status'] == 'success' else "❌"
            self.logger.info(f"  {status_emoji} [{result['index']}] {result['test_point']} | {result['samples']} samples")
        
        self.logger.info(f"{'='*70}\n")
        
        return all(r['status'] == 'success' for r in results)

# ============================================================================
# HELPER FUNCTIONS (for creating test suites)
# ============================================================================

def create_speed_sweep(
    speeds: List[int],
    duration_ms: int,
    max_force_N: float,
    description: str = ""
) -> TestSuite:
    """Create test suite with varying speeds."""
    test_points = [
        TestPoint(
            speed=s,
            duration_ms=duration_ms,
            max_force_N=max_force_N,
            description=f"{description} Speed={s}" if description else None
        )
        for s in speeds
    ]
    return TestSuite(
        name=f"Speed Sweep ({min(speeds)}-{max(speeds)})",
        test_points=test_points
    )

def create_force_sweep(
    speed: int,
    duration_ms: int,
    max_forces: List[float],
    description: str = ""
) -> TestSuite:
    """Create test suite with varying force limits."""
    test_points = [
        TestPoint(
            speed=speed,
            duration_ms=duration_ms,
            max_force_N=f,
            description=f"{description} MaxForce={f}" if description else None
        )
        for f in max_forces
    ]
    return TestSuite(
        name=f"Force Sweep ({min(max_forces)}-{max(max_forces)}N)",
        test_points=test_points
    )

def create_parametric_grid(
    speeds: List[int],
    max_forces: List[float],
    duration_ms: int
) -> TestSuite:
    """Create Cartesian product of speeds × forces."""
    test_points = [
        TestPoint(
            speed=s,
            duration_ms=duration_ms,
            max_force_N=f,
            description=f"s={s} f={f}N"
        )
        for s in speeds
        for f in max_forces
    ]
    return TestSuite(
        name=f"Parametric Grid {len(speeds)}×{len(max_forces)}",
        test_points=test_points
    )

# ============================================================================
# SETUP LOGGING
# ============================================================================

def setup_logging() -> None:
    """Configure logging."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)-5s] %(message)s',
        datefmt='%H:%M:%S'
    )

# ============================================================================
# MAIN (Example usage)
# ============================================================================

if __name__ == "__main__":
    setup_logging()
    
    # Example 1: Speed sweep (100, 200, 300, 400, 500)
    speeds = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500]
    suite1 = create_speed_sweep(
        speeds=speeds,
        duration_ms=100000,
        max_force_N=10.0,
        description="Test Speed Variation"
    )
    
    # Example 2: Force sweep (10, 20, 30, 40, 50 N)
    forces = [10.0, 20.0, 30.0, 40.0, 50.0]
    suite2 = create_force_sweep(
        speed=400,
        duration_ms=10000,
        max_forces=forces,
        description="Test Force Variation"
    )
    
    # Example 3: 2D grid (3 speeds × 3 forces = 9 tests)
    suite3 = create_parametric_grid(
        speeds=speeds,                 #[200, 300, 400],
        max_forces=[10.0, 50.0],
        duration_ms=100000
    )
    
    # Run automation
    automation = StepResponseAutomation(output_dir=Path.cwd() / "test_results")
    
    if automation.connect():
        try:
            # Run one or multiple suites
            #automation.run_test_suite(suite1)
            #automation.run_test_suite(suite2)
            automation.run_test_suite(suite3)
        finally:
            automation.disconnect()
    else:
        print("Failed to connect to device")
