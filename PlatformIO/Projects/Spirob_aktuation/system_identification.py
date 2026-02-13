#!/usr/bin/env python3
"""
SpiRob System Identification - Object-Oriented Refactoring
Captures 600Hz sensor data (force, position) during step responses
for PID tuning via state machine-driven workflow.
"""

import serial
import struct
import time
import sys
import select
import logging
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Tuple, Optional
from pathlib import Path
from datetime import datetime

import numpy as np
import polars as pl
import matplotlib.pyplot as plt

# ============================================================================
# CONFIGURATION
# ============================================================================

@dataclass(frozen=True)
class SpiRobConfig:
    """Immutable configuration for SpiRob serial communication."""
    serial_port: str = '/dev/ttyUSB0'
    baudrate: int = 460800
    timeout: float = 0.001
    status_header: bytes = b'\xaa\x55'          # Normal status packet
    step_end_header: bytes = b'\xbb\x66'       # Step-end signal (2 bytes)
    struct_format: str = '<I f f'  # uint32, float32, float32
    struct_size: int = 12
    sync_timeout: float = 0.1
    output_dir: Path = field(default_factory=lambda: Path.cwd())

# ============================================================================
# DATA MODELS
# ============================================================================

@dataclass
class SensorData:
    """Single sensor reading from ESP32."""
    timestamp_s: float          # System time (Python)
    timestamp_us: int           # MCU microseconds
    force_N: float              # Tendon force [Newtons]
    cable_length_mm: float      # Cable length [millimeters]

@dataclass
class StepConfig:
    """Step response test configuration."""
    speed: int              # Motor speed command
    duration_ms: int        # Step duration in milliseconds
    
    @property
    def duration_s(self) -> float:
        """Duration in seconds."""
        return self.duration_ms / 1000.0

@dataclass
class ExperimentMeta:
    """Metadata for step response experiment."""
    step_speed: int
    duration_ms: int
    max_force: Optional[float]      # From 'f' command
    end_reason: str                 # 'force_limit', 'timeout', or 'unknown'
    actual_duration_s: float
    sample_count: int
    sample_rate_hz: float
    timestamp: str                  # ISO format

# ============================================================================
# STATE MACHINE
# ============================================================================

class RecorderState(Enum):
    """State machine states for data acquisition."""
    IDLE = auto()               # Monitoring live data
    RECORDING = auto()          # Capturing step response (buffer only)
    WAIT_END_SIGNAL = auto()    # Waiting for SINGLE end signal after step timeout
    ANALYSIS = auto()           # Processing & plotting data

# ============================================================================
# SERIAL CONNECTION
# ============================================================================

class SerialConnection:
    """Manages serial communication with ESP32."""
    
    def __init__(self, config: SpiRobConfig):
        self.config = config
        self.serial: Optional[serial.Serial] = None
        self.logger = logging.getLogger(__name__)
    
    def connect(self) -> None:
        """Establish serial connection and flush buffers."""
        try:
            self.serial = serial.Serial(
                self.config.serial_port,
                self.config.baudrate,
                timeout=self.config.timeout
            )
            self.logger.info(f"Connected to {self.config.serial_port} @ {self.config.baudrate} baud")
            time.sleep(0.1)  # MCU stabilization
            self.serial.reset_input_buffer()
        except serial.SerialException as e:
            self.logger.error(f"Failed to connect: {e}")
            raise
    def clear_input_buffer(self) -> None:
        """Flush serial input buffer."""
        if self.serial and self.serial.is_open:
            self.serial.reset_input_buffer()
    
    def disconnect(self) -> None:
        """Close serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.logger.info("Serial connection closed")
    
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
    
    def sync_to_header(self) -> Optional[str]:
        """Synchronize to binary protocol header. Returns 'status' or 'step_end' or None."""
        if not self.serial:
            return None
        sync_buf = b''
        timeout_start = time.time()
        
        while time.time() - timeout_start < self.config.sync_timeout:
            byte = self.serial.read(1)
            if not byte:
                continue
            
            sync_buf += byte
            
            # Keep buffer size appropriate for headers (2 bytes)
            if len(sync_buf) > 2:
                sync_buf = sync_buf[-2:]
            
            # Check for step-end header (0xBB 0x66)
            if len(sync_buf) == 2 and sync_buf == self.config.step_end_header:
                self.logger.warning(f"🎯 STEP END HEADER! [{sync_buf.hex(' ')}]")
                return 'step_end'
            
            # Check for status header (0xAA 0x55)
            if len(sync_buf) == 2 and sync_buf == self.config.status_header:
                return 'status'
        
        return None
    
    def is_data_ready(self) -> bool:
        """Check if serial data is available (non-blocking)."""
        if not self.serial:
            return False
        ready, _, _ = select.select([self.serial], [], [], 0)
        return bool(ready)

# ============================================================================
# DATA BUFFER
# ============================================================================

class DataBuffer:
    """Manages sensor data collection and export."""
    
    def __init__(self):
        self.data: List[SensorData] = []
        self.logger = logging.getLogger(__name__)
    
    def append(self, sample: SensorData) -> None:
        """Add sensor reading to buffer."""
        self.data.append(sample)
    
    def clear(self) -> None:
        """Reset buffer."""
        self.data.clear()
    
    def __len__(self) -> int:
        return len(self.data)
    
    def to_polars_df(self) -> pl.DataFrame:
        """Convert buffer to Polars DataFrame."""
        if not self.data:
            raise ValueError("Buffer is empty")
        
        return pl.DataFrame({
            "timestamp_s": [d.timestamp_s for d in self.data],
            "timestamp_us": [d.timestamp_us for d in self.data],
            "force_N": [d.force_N for d in self.data],
            "cable_length_mm": [d.cable_length_mm for d in self.data]
        })
    
    def save_parquet(self, filepath: Path) -> None:
        """Save buffer as Parquet file."""
        df = self.to_polars_df()
        df.write_parquet(filepath)
        self.logger.info(f"Saved {len(df)} samples to {filepath}")
    
    def save_metadata(self, filepath: Path, meta: ExperimentMeta) -> None:
        """Save experiment metadata as separate Parquet file."""
        meta_df = pl.DataFrame([{
            "step_speed": meta.step_speed,
            "duration_ms": meta.duration_ms,
            "max_force": meta.max_force,
            "end_reason": meta.end_reason,
            "actual_duration_s": meta.actual_duration_s,
            "sample_count": meta.sample_count,
            "sample_rate_hz": meta.sample_rate_hz,
            "timestamp": meta.timestamp
        }])
        meta_df.write_parquet(filepath)
        self.logger.info(f"Saved metadata to {filepath}")
    
    def get_sample_rate(self, start_time: float, end_time: float) -> float:
        """Calculate effective sample rate [Hz]."""
        duration = end_time - start_time
        return len(self.data) / duration if duration > 0 else 0.0

# ============================================================================
# LIVE MONITOR
# ============================================================================

class LiveMonitor:
    """Displays real-time sensor data during IDLE state."""
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
    
    def print_sample(self, sample: SensorData) -> None:
        """Print live data to console (overwrite line)."""
        print(f"\r📡 {sample.timestamp_us:10d}us | {sample.force_N:7.3f}N | {sample.cable_length_mm:7.3f}mm", 
              end='', flush=True)

# ============================================================================
# STEP CONTROLLER
# ============================================================================

class StepController:
    """Orchestrates step response tests and data analysis."""
    
    def __init__(self, config: SpiRobConfig):
        self.config = config
        self.current_step: Optional[StepConfig] = None
        self.logger = logging.getLogger(__name__)
    
    @staticmethod
    def parse_step_command(cmd: str) -> Optional[StepConfig]:
        """Parse 'step <speed> <ms>' command into StepConfig."""
        parts = cmd.split()
        if len(parts) != 3 or parts[0] != 'step':
            return None
        try:
            return StepConfig(speed=int(parts[1]), duration_ms=int(parts[2]))
        except ValueError:
            return None
    
    @staticmethod
    def parse_force_command(cmd: str) -> Optional[float]:
        """Parse 'f <max_force>' command."""
        parts = cmd.split()
        if len(parts) != 2 or parts[0] != 'f':
            return None
        try:
            return float(parts[1])
        except ValueError:
            return None
    
    def generate_plots(self, buffer: DataBuffer, step: StepConfig) -> Path:
        """Create 3-panel plot (Force, Cable Length, Cable Velocity) and save."""
        df = buffer.to_polars_df()
        time_s = df["timestamp_s"].to_numpy() - df["timestamp_s"][0]
        
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        
        # Force plot
        axes[0].plot(time_s, df["force_N"].to_numpy(), 'b-', linewidth=0.8)
        axes[0].set_ylabel("Force [N]", fontsize=11)
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title(f"SpiRob Step Response | Speed={step.speed} | Duration={step.duration_ms}ms", 
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
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plot_path = self.config.output_dir / f"spirob_step_{timestamp}.png"
        plt.savefig(plot_path, dpi=200, bbox_inches='tight')
        plt.show()
        
        self.logger.info(f"Plot saved: {plot_path}")
        return plot_path

# ============================================================================
# STATE MACHINE RECORDER
# ============================================================================

class SpiRobRecorder:
    """Main state machine for SpiRob data acquisition."""
    
    def __init__(self, config: SpiRobConfig):
        self.config = config
        self.state = RecorderState.IDLE
        
        # Components
        self.serial = SerialConnection(config)
        self.buffer = DataBuffer()
        self.monitor = LiveMonitor()
        self.controller = StepController(config)
        
        # Recording state
        self.recording_start: float = 0.0
        self.recording_end: float = 0.0
        self.wait_end_deadline: float = 0.0  # Deadline for end signal (5s total)
        self.max_force: Optional[float] = None
        self.end_reason: str = 'unknown'
        
        self.logger = logging.getLogger(__name__)
    
    def connect(self) -> None:
        """Initialize serial connection."""
        self.serial.connect()
        self.logger.info("🔌 Ready | 'step <speed> <ms>' to record | 'f <force>' to set limit | 'r' to return to 0 | 'q' to quit")
    
    def disconnect(self) -> None:
        """Cleanup resources."""
        self.serial.disconnect()
    
    def process_serial_data(self) -> None:
        """Read and process incoming binary sensor data."""
        if not self.serial.is_data_ready():
            return
        
        header_type = self.serial.sync_to_header()
        if not header_type:
            return
        
        # Normal status packet (IDLE or RECORDING states)
        if header_type == 'status':
            data = self.serial.read_exact(self.config.struct_size)
            if not data:
                return
            
            try:
                timestamp_us, force_N, cable_length_mm = struct.unpack(self.config.struct_format, data)
                sample = SensorData(
                    timestamp_s=time.time(),
                    timestamp_us=timestamp_us,
                    force_N=force_N,
                    cable_length_mm=cable_length_mm
                )
                
                # State-dependent handling
                if self.state == RecorderState.IDLE:
                    self.monitor.print_sample(sample)
                elif self.state == RecorderState.RECORDING:
                    self.buffer.append(sample)
                # WAIT_END_SIGNAL state: ignore status packets, only looking for step_end signal
                    
            except struct.error as e:
                self.logger.warning(f"Unpack error: {e}")
        
        # Step-end signal (accept during RECORDING or WAIT_END_SIGNAL)
        elif header_type == 'step_end':
            if self.state in [RecorderState.RECORDING, RecorderState.WAIT_END_SIGNAL]:
                self.logger.warning("✅ STEP END detected - finishing recording")
                self.end_reason = 'step_completed'
                self._finish_recording()
            else:
                self.logger.warning(f"⚠️ Unexpected step_end signal in state: {self.state} (ignoring)")
    
    def process_user_input(self) -> bool:
        """Handle user commands (non-blocking). Returns False to quit."""
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            return True
        
        cmd = sys.stdin.readline().strip()
        
        if cmd.lower() == 'q':
            return False
        
        # Return to rope length 0 command
        if cmd.lower() == 'r':
            self.serial.write_command('r')
            self.logger.info("→ r | Returning to rope length 0...")
            return True
        
        # Step command: step <speed> <ms>
        step = StepController.parse_step_command(cmd)
        if step:
            self._start_step_recording(step)
            return True
        
        # Force limit command: f <max_force>
        max_force = StepController.parse_force_command(cmd)
        if max_force is not None:
            self.max_force = max_force
            self.serial.write_command(cmd)
            self.logger.info(f"→ {cmd} | Max force set to {max_force}N")
            return True
        
        # Forward other commands to ESP32
        self.serial.write_command(cmd)
        self.logger.info(f"→ {cmd}")
        
        return True
    
    def _start_step_recording(self, step: StepConfig) -> None:
        """Transition to RECORDING state and initiate step test."""
        if self.state != RecorderState.IDLE:
            self.logger.warning("Cannot start recording: not in IDLE state")
            return
        
        # Send command to ESP32
        self.serial.write_command(f"step {step.speed} {step.duration_ms}")
        
        # Prepare recording
        self.buffer.clear()
        self.controller.current_step = step
        self.recording_start = time.time()
        self.recording_end = self.recording_start + step.duration_s + 0.5  # Step duration + 500ms grace
        self.wait_end_deadline = self.recording_start + step.duration_s + 5.0  # 5s total timeout for end signal
        self.end_reason = 'unknown'  # Will be updated by end signal
        
        # State transition
        self.state = RecorderState.RECORDING
        
        max_force_str = f" | MaxForce={self.max_force}N" if self.max_force else ""
        self.logger.info(f"🎬 RECORDING | Speed={step.speed} | Duration={step.duration_ms}ms{max_force_str}")
    
    def update_state(self) -> None:
        """Check for state transitions."""
        if self.state == RecorderState.RECORDING:
            # After step duration + grace period: transition to WAIT_END_SIGNAL
            if time.time() >= self.recording_end:
                self.logger.info("⏱️ Recording timeout - waiting for end signal...")
                self.state = RecorderState.WAIT_END_SIGNAL
        
        elif self.state == RecorderState.WAIT_END_SIGNAL:
            # If end signal not received within 5s total: timeout fallback
            if time.time() >= self.wait_end_deadline:
                self.logger.warning("❌ End signal timeout (5s exceeded) - proceeding with analysis")
                self.end_reason = 'missing_end_signal'
                self._finish_recording()
        
        elif self.state == RecorderState.ANALYSIS:
            self._perform_analysis()
    
    def _finish_recording(self) -> None:
        """Transition from RECORDING to ANALYSIS."""
        self.state = RecorderState.ANALYSIS
        
        sample_rate = self.buffer.get_sample_rate(self.recording_start, time.time())
        self.logger.info(f"✅ Recording complete | {len(self.buffer)} samples @ {sample_rate:.0f}Hz | End: {self.end_reason}")
    
    def _perform_analysis(self) -> None:
        """Process data, save, plot, then return to IDLE."""
        if len(self.buffer) < 2:
            self.logger.error("❌ Insufficient data points")
            self.state = RecorderState.IDLE
            return
        
        # Calculate metrics
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        actual_duration = time.time() - self.recording_start
        sample_rate = self.buffer.get_sample_rate(self.recording_start, time.time())
        
        # Create metadata
        step = self.controller.current_step
        if step:
            meta = ExperimentMeta(
                step_speed=step.speed,
                duration_ms=step.duration_ms,
                max_force=self.max_force,
                end_reason=self.end_reason,
                actual_duration_s=actual_duration,
                sample_count=len(self.buffer),
                sample_rate_hz=sample_rate,
                timestamp=datetime.now().isoformat()
            )
        
        # Save data
        parquet_path = self.config.output_dir / f"spirob_step_{timestamp_str}.parquet"
        meta_path = self.config.output_dir / f"spirob_step_{timestamp_str}_meta.parquet"
        self.buffer.save_parquet(parquet_path)
        self.buffer.save_metadata(meta_path, meta)
        
        # Generate plots
        if step:
            self.controller.generate_plots(self.buffer, step)
            self.buffer.clear()
            self.serial.clear_input_buffer()
        
        # Reset to IDLE
        self.state = RecorderState.IDLE
        self.logger.info("↻ Ready for next command\n")
    
    def run(self) -> None:
        """Main event loop with state machine."""
        try:
            self.connect()
            
            while True:
                # Process incoming data (600Hz non-blocking)
                self.process_serial_data()
                
                # Handle user input
                if not self.process_user_input():
                    break
                
                # State transitions
                self.update_state()
                
                # Minimal sleep to prevent CPU thrashing
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            self.logger.info("\n👋 Interrupted by user")
        except Exception as e:
            self.logger.error(f"Unexpected error: {e}", exc_info=True)
        finally:
            self.disconnect()

# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def setup_logging() -> None:
    """Configure logging with timestamp and colored output."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)-5s] %(message)s',
        datefmt='%H:%M:%S'
    )

def main() -> None:
    """Application entry point."""
    setup_logging()
    
    config = SpiRobConfig()
    recorder = SpiRobRecorder(config)
    recorder.run()

if __name__ == "__main__":
    main()
