#!/usr/bin/env python3
"""
Step Response Data Analysis Framework
Loads, filters, trims and analyzes measurement data from test_results/
"""

import logging
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional, Tuple
from datetime import datetime

import numpy as np
import polars as pl
from scipy import signal, stats
import matplotlib.pyplot as plt

# ============================================================================
# CONFIGURATION
# ============================================================================

DEFAULT_TEST_RESULTS_DIR = Path.cwd() / "test_results"

# Outlier detection methods
OUTLIER_METHODS = ["zscore", "iqr", "isolation_forest", "mahalanobis"]

# ============================================================================
# LOGGING SETUP
# ============================================================================

def setup_logging(level=logging.INFO) -> None:
    """Configure logging for analysis."""
    logging.basicConfig(
        level=level,
        format='%(asctime)s [%(levelname)-5s] %(message)s',
        datefmt='%H:%M:%S'
    )

logger = logging.getLogger(__name__)

# ============================================================================
# DATA LOADING
# ============================================================================

class DataLoader:
    """Load measurement data from parquet files."""
    
    def __init__(self, data_dir: Path = DEFAULT_TEST_RESULTS_DIR):
        self.data_dir = Path(data_dir)
        self.logger = logging.getLogger(__name__)
    
    def load_all_tests(self) -> List[Tuple[str, pl.DataFrame]]:
        """Load all parquet files from test_results directory."""
        parquet_files = sorted(self.data_dir.glob("*.parquet"))
        
        if not parquet_files:
            self.logger.warning(f"Keine Parquet-Dateien in {self.data_dir} gefunden")
            return []
        
        loaded_data = []
        for filepath in parquet_files:
            try:
                df = pl.read_parquet(filepath)
                loaded_data.append((filepath.name, df))
                self.logger.info(f"✅ Geladen: {filepath.name} ({len(df)} Samples)")
            except Exception as e:
                self.logger.error(f"❌ Fehler beim Laden von {filepath.name}: {e}")
        
        return loaded_data
    
    def load_single_test(self, filename: str) -> Optional[pl.DataFrame]:
        """Load specific parquet file."""
        filepath = self.data_dir / filename
        
        if not filepath.exists():
            self.logger.error(f"Datei nicht gefunden: {filepath}")
            return None
        
        try:
            df = pl.read_parquet(filepath)
            self.logger.info(f"✅ Geladen: {filename} ({len(df)} Samples)")
            return df
        except Exception as e:
            self.logger.error(f"❌ Fehler beim Laden: {e}")
            return None
    
    def load_tests_by_force(self, force_N: float) -> List[Tuple[str, pl.DataFrame]]:
        """Load all parquet files that match a specific force value (from filename)."""
        parquet_files = sorted(self.data_dir.glob("*.parquet"))
        force_str = f"{force_N:.1f}N"
        
        loaded_data = []
        for filepath in parquet_files:
            # Check if force value is in filename
            if force_str in filepath.name:
                try:
                    df = pl.read_parquet(filepath)
                    loaded_data.append((filepath.name, df))
                    self.logger.info(f"✅ Geladen: {filepath.name} ({len(df)} Samples)")
                except Exception as e:
                    self.logger.error(f"❌ Fehler beim Laden von {filepath.name}: {e}")
        
        if not loaded_data:
            self.logger.warning(f"Keine Tests mit {force_str} gefunden")
        else:
            self.logger.info(f"✅ {len(loaded_data)} Tests mit {force_str} geladen")
        
        return loaded_data

# ============================================================================
# DATA TRIMMING
# ============================================================================

class DataTrimmier:
    """Trim measurement data (remove beginning/end)."""
    
    @staticmethod
    def trim_by_time(
        df: pl.DataFrame,
        trim_start_s: float = 0.0,
        trim_end_s: float = 0.0
    ) -> pl.DataFrame:
        """
        Trim data by time duration from start and end.
        
        Args:
            df: Input DataFrame
            trim_start_s: Seconds to remove from beginning
            trim_end_s: Seconds to remove from end
        
        Returns:
            Trimmed DataFrame
        """
        if trim_start_s == 0.0 and trim_end_s == 0.0:
            return df
        
        # Get timestamp range
        t_min = df["timestamp_s"].min()
        t_max = df["timestamp_s"].max()
        
        # Calculate cut points
        t_start = t_min + trim_start_s
        t_end = t_max - trim_end_s
        
        # Filter
        trimmed = df.filter(
            (pl.col("timestamp_s") >= t_start) & 
            (pl.col("timestamp_s") <= t_end)
        )
        
        logger.info(f"Getrimmt: {len(df)} → {len(trimmed)} Samples (Start: {trim_start_s}s, Ende: {trim_end_s}s)")
        return trimmed
    
    @staticmethod
    def trim_by_samples(
        df: pl.DataFrame,
        trim_start_samples: int = 0,
        trim_end_samples: int = 0
    ) -> pl.DataFrame:
        """
        Trim data by number of samples from start and end.
        
        Args:
            df: Input DataFrame
            trim_start_samples: Number of samples to remove from beginning
            trim_end_samples: Number of samples to remove from end
        
        Returns:
            Trimmed DataFrame
        """
        if trim_start_samples == 0 and trim_end_samples == 0:
            return df
        
        n = len(df)
        start_idx = trim_start_samples
        end_idx = n - trim_end_samples
        
        trimmed = df[start_idx:end_idx]
        
        logger.info(f"Getrimmt: {n} → {len(trimmed)} Samples (Start: {trim_start_samples}, Ende: {trim_end_samples})")
        return trimmed
    
    @staticmethod
    def extract_window(
        df: pl.DataFrame,
        skip_start_samples: int = 0,
        num_samples_to_keep: int = None
    ) -> pl.DataFrame:
        """
        Extract a window of data: skip N samples from start, then keep M samples.
        
        Args:
            df: Input DataFrame
            skip_start_samples: Number of samples to skip at the beginning
            num_samples_to_keep: Number of samples to keep after skipping.
                                If None, keeps all remaining samples
        
        Returns:
            DataFrame with extracted window
        
        Example:
            df = load_data()  # 10000 samples
            windowed = trimmer.extract_window(df, skip_start_samples=100, num_samples_to_keep=5000)
            # Result: 5000 samples (skipped first 100, kept next 5000, removed rest)
        """
        n = len(df)
        
        # Validate skip_start_samples
        if skip_start_samples < 0:
            logger.warning(f"skip_start_samples kann nicht negativ sein ({skip_start_samples}), setze auf 0")
            skip_start_samples = 0
        if skip_start_samples >= n:
            logger.error(f"skip_start_samples ({skip_start_samples}) >= Gesamtanzahl Samples ({n})")
            return df.slice(0, 0)  # Return empty
        
        # Determine end index
        if num_samples_to_keep is None:
            end_idx = n
            kept_samples = n - skip_start_samples
        else:
            if num_samples_to_keep <= 0:
                logger.warning(f"num_samples_to_keep muss > 0 sein ({num_samples_to_keep}), setze auf verfügbare Samples")
                num_samples_to_keep = n - skip_start_samples
            
            end_idx = min(skip_start_samples + num_samples_to_keep, n)
            kept_samples = end_idx - skip_start_samples
        
        # Extract window
        windowed = df[skip_start_samples:end_idx]
        
        logger.info(
            f"Fenster extrahiert: {n} → {len(windowed)} Samples "
            f"(überspringe {skip_start_samples}, behalte {kept_samples})"
        )
        return windowed

# ============================================================================
# OUTLIER DETECTION & REMOVAL
# ============================================================================

class OutlierDetector:
    """Detect and remove outliers from measurement data."""
    
    @staticmethod
    def zscore_filter(
        df: pl.DataFrame,
        column: str = "force_N",
        threshold: float = 3.0
    ) -> Tuple[pl.DataFrame, pl.DataFrame]:
        """
        Filter outliers using Z-score method.
        
        Args:
            df: Input DataFrame
            column: Column to check for outliers
            threshold: Z-score threshold (default: 3.0)
        
        Returns:
            (filtered_df, outliers_df)
        """
        values = df[column].to_numpy()
        z_scores = np.abs(stats.zscore(values))
        
        outlier_mask = z_scores > threshold
        
        filtered = df.filter(~pl.Series(outlier_mask))
        outliers = df.filter(pl.Series(outlier_mask))
        
        logger.info(f"Z-Score Filter: {len(outliers)} Ausreißer gefunden (threshold={threshold})")
        return filtered, outliers
    
    @staticmethod
    def iqr_filter(
        df: pl.DataFrame,
        column: str = "force_N",
        k: float = 1.5
    ) -> Tuple[pl.DataFrame, pl.DataFrame]:
        """
        Filter outliers using Interquartile Range (IQR) method.
        
        Args:
            df: Input DataFrame
            column: Column to check for outliers
            k: IQR multiplier (default: 1.5)
        
        Returns:
            (filtered_df, outliers_df)
        """
        values = df[column].to_numpy()
        q1 = np.percentile(values, 25)
        q3 = np.percentile(values, 75)
        iqr = q3 - q1
        
        lower_bound = q1 - k * iqr
        upper_bound = q3 + k * iqr
        
        outlier_mask = (values < lower_bound) | (values > upper_bound)
        
        filtered = df.filter(~pl.Series(outlier_mask))
        outliers = df.filter(pl.Series(outlier_mask))
        
        logger.info(f"IQR Filter: {len(outliers)} Ausreißer gefunden (k={k})")
        return filtered, outliers
    
    @staticmethod
    def isolation_forest_filter(
        df: pl.DataFrame,
        columns: List[str] = ["force_N", "cable_length_mm"],
        contamination: float = 0.05
    ) -> Tuple[pl.DataFrame, pl.DataFrame]:
        """
        Filter outliers using Isolation Forest algorithm.
        
        Args:
            df: Input DataFrame
            columns: Columns to use for outlier detection
            contamination: Expected proportion of outliers (default: 0.05)
        
        Returns:
            (filtered_df, outliers_df)
        """
        from sklearn.ensemble import IsolationForest
        
        data = df.select(columns).to_numpy()
        
        iso_forest = IsolationForest(contamination=contamination, random_state=42)
        predictions = iso_forest.fit_predict(data)
        
        outlier_mask = predictions == -1
        
        filtered = df.filter(~pl.Series(outlier_mask))
        outliers = df.filter(pl.Series(outlier_mask))
        
        logger.info(f"Isolation Forest: {len(outliers)} Ausreißer gefunden (contamination={contamination})")
        return filtered, outliers
    
    @staticmethod
    def mahalanobis_filter(
        df: pl.DataFrame,
        columns: List[str] = ["force_N", "cable_length_mm"],
        threshold: float = 3.0
    ) -> Tuple[pl.DataFrame, pl.DataFrame]:
        """
        Filter outliers using Mahalanobis distance.
        
        Args:
            df: Input DataFrame
            columns: Columns to use for outlier detection
            threshold: Distance threshold (default: 3.0)
        
        Returns:
            (filtered_df, outliers_df)
        """
        data = df.select(columns).to_numpy()
        mean = np.mean(data, axis=0)
        cov = np.cov(data.T)
        
        if np.linalg.matrix_rank(cov) < len(columns):
            logger.warning("Kovarianzmatrix ist singulär, verwende pinv()")
            cov_inv = np.linalg.pinv(cov)
        else:
            cov_inv = np.linalg.inv(cov)
        
        distances = np.sqrt(np.sum((data - mean) @ cov_inv * (data - mean), axis=1))
        outlier_mask = distances > threshold
        
        filtered = df.filter(~pl.Series(outlier_mask))
        outliers = df.filter(pl.Series(outlier_mask))
        
        logger.info(f"Mahalanobis Filter: {len(outliers)} Ausreißer gefunden (threshold={threshold})")
        return filtered, outliers

# ============================================================================
# SMOOTHING / FILTERING
# ============================================================================

class SignalFilter:
    """Apply signal processing filters to smooth data."""
    
    @staticmethod
    def moving_average(
        df: pl.DataFrame,
        column: str,
        window_size: int = 10
    ) -> pl.DataFrame:
        """
        Apply moving average filter.
        
        Args:
            df: Input DataFrame
            column: Column to filter
            window_size: Size of moving window
        
        Returns:
            DataFrame with filtered column
        """
        values = df[column].to_numpy()
        filtered = np.convolve(values, np.ones(window_size) / window_size, mode='same')
        
        return df.with_columns(pl.Series(f"{column}_smoothed", filtered))
    
    @staticmethod
    def savitzky_golay_filter(
        df: pl.DataFrame,
        column: str,
        window_length: int = 11,
        polyorder: int = 2
    ) -> pl.DataFrame:
        """
        Apply Savitzky-Golay filter (better for preserving features).
        
        Args:
            df: Input DataFrame
            column: Column to filter
            window_length: Window size (must be odd)
            polyorder: Polynomial order
        
        Returns:
            DataFrame with filtered column
        """
        # Ensure window length is odd
        if window_length % 2 == 0:
            window_length += 1
        
        values = df[column].to_numpy()
        filtered = signal.savgol_filter(values, window_length, polyorder)
        
        return df.with_columns(pl.Series(f"{column}_smoothed", filtered))
    
    @staticmethod
    def butterworth_filter(
        df: pl.DataFrame,
        column: str,
        cutoff_hz: float,
        sample_rate_hz: float,
        order: int = 4
    ) -> pl.DataFrame:
        """
        Apply Butterworth low-pass filter.
        
        Args:
            df: Input DataFrame
            column: Column to filter
            cutoff_hz: Cutoff frequency in Hz
            sample_rate_hz: Sampling rate in Hz
            order: Filter order
        
        Returns:
            DataFrame with filtered column
        """
        nyquist = sample_rate_hz / 2
        normalized_cutoff = cutoff_hz / nyquist
        
        if normalized_cutoff >= 1.0:
            logger.warning(f"Cutoff-Frequenz ({cutoff_hz}Hz) >= Nyquist-Frequenz ({nyquist}Hz)")
            return df
        
        b, a = signal.butter(order, normalized_cutoff, btype='low')
        values = df[column].to_numpy()
        filtered = signal.filtfilt(b, a, values)
        
        return df.with_columns(pl.Series(f"{column}_filtered", filtered))
    
    @staticmethod
    def calculate_velocity_robust(
        df: pl.DataFrame,
        position_column: str = "cable_length_mm",
        time_column: str = "timestamp_s",
        window_samples: int = 50,
        method: str = "moving_window"
    ) -> np.ndarray:
        """
        Calculate velocity robustly over longer time windows to reduce noise.
        
        Args:
            df: Input DataFrame
            position_column: Column with position data
            time_column: Column with time data
            window_samples: Number of samples to use for velocity calculation
            method: Calculation method:
                - "moving_window": Central difference over window
                - "savitzky_golay": Smoothed derivative
                - "polyfit": Polynomial fit over window
        
        Returns:
            Array of velocity values
        """
        position = df[position_column].to_numpy()
        time = df[time_column].to_numpy()
        n = len(position)
        
        if method == "moving_window":
            # Use central difference over a window
            velocity = np.zeros(n)
            half_window = window_samples // 2
            
            for i in range(half_window, n - half_window):
                # Position difference over full window
                delta_pos = position[i + half_window] - position[i - half_window]
                delta_time = time[i + half_window] - time[i - half_window]
                
                if delta_time > 0:
                    velocity[i] = delta_pos / delta_time
            
            # Handle edges with forward/backward difference
            for i in range(half_window):
                if i > 0 and time[i] - time[i-1] > 0:
                    velocity[i] = (position[i] - position[i-1]) / (time[i] - time[i-1])
            
            for i in range(n - half_window, n):
                if i < n - 1 and time[i+1] - time[i] > 0:
                    velocity[i] = (position[i+1] - position[i]) / (time[i+1] - time[i])
            
            logger.info(f"Geschwindigkeit berechnet (Moving Window: {window_samples} Samples)")
        
        elif method == "savitzky_golay":
            # Use Savitzky-Golay derivative (smooths while taking derivative)
            # Ensure window is odd
            if window_samples % 2 == 0:
                window_samples += 1
            
            # Polyfit order (typically 2-3 for derivative)
            polyorder = min(3, window_samples - 1)
            
            # Calculate derivative using Savitzky-Golay
            try:
                velocity = signal.savgol_filter(position, window_samples, polyorder, deriv=1)
                # Convert from position/sample to position/time
                # Get average dt
                avg_dt = (time[-1] - time[0]) / (n - 1)
                velocity = velocity / avg_dt
                logger.info(f"Geschwindigkeit berechnet (Savitzky-Golay: {window_samples} Samples, polyorder={polyorder})")
            except Exception as e:
                logger.warning(f"Savitzky-Golay fehlgeschlagen: {e}, fallback zu Moving Window")
                return SignalFilter.calculate_velocity_robust(
                    df, position_column, time_column, window_samples, method="moving_window"
                )
        
        elif method == "polyfit":
            # Fit polynomial over window and use derivative
            velocity = np.zeros(n)
            half_window = window_samples // 2
            
            for i in range(half_window, n - half_window):
                # Get window around point
                window_start = max(0, i - half_window)
                window_end = min(n, i + half_window + 1)
                
                window_pos = position[window_start:window_end]
                window_time = time[window_start:window_end]
                
                # Fit 2nd order polynomial
                coeffs = np.polyfit(window_time, window_pos, 2)
                # Derivative: d/dt = 2*a*t + b
                velocity[i] = 2 * coeffs[0] * time[i] + coeffs[1]
            
            # Handle edges
            for i in range(half_window):
                if i > 0 and time[i] - time[i-1] > 0:
                    velocity[i] = (position[i] - position[i-1]) / (time[i] - time[i-1])
            
            for i in range(n - half_window, n):
                if i < n - 1 and time[i+1] - time[i] > 0:
                    velocity[i] = (position[i+1] - position[i]) / (time[i+1] - time[i])
            
            logger.info(f"Geschwindigkeit berechnet (Polyfit: {window_samples} Samples)")
        
        else:
            raise ValueError(f"Unbekannte Methode: {method}")
        
        return velocity
    
    @staticmethod
    def filter_velocity_outliers(
        velocity: np.ndarray,
        method: str = "zscore",
        threshold: float = 3.0,
        window_size: int = 5
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Filter outliers in velocity data.
        
        Args:
            velocity: Velocity array
            method: Outlier detection method:
                - "zscore": Z-score based (default)
                - "iqr": Interquartile Range based
                - "gradient": Large jumps compared to neighbors
                - "median_diff": Difference to median filter
            threshold: Detection threshold (interpretation depends on method)
            window_size: Window size for local comparisons
        
        Returns:
            (filtered_velocity, outlier_mask)
        """
        n = len(velocity)
        outlier_mask = np.zeros(n, dtype=bool)
        
        if method == "zscore":
            # Z-score method
            z_scores = np.abs(stats.zscore(velocity))
            outlier_mask = z_scores > threshold
            logger.info(f"Velocity Z-Score Filter: {np.sum(outlier_mask)} Ausreißer (threshold={threshold})")
        
        elif method == "iqr":
            # Interquartile Range method
            q1 = np.percentile(velocity, 25)
            q3 = np.percentile(velocity, 75)
            iqr = q3 - q1
            
            lower_bound = q1 - threshold * iqr
            upper_bound = q3 + threshold * iqr
            
            outlier_mask = (velocity < lower_bound) | (velocity > upper_bound)
            logger.info(f"Velocity IQR Filter: {np.sum(outlier_mask)} Ausreißer (k={threshold})")
        
        elif method == "gradient":
            # Detect large jumps in velocity (second derivative)
            # Calculate velocity change between consecutive points
            vel_diff = np.abs(np.diff(velocity))
            
            # Find median absolute deviation
            median_diff = np.median(vel_diff)
            mad = np.median(np.abs(vel_diff - median_diff))
            
            # Outliers are points where velocity change > threshold * MAD
            # (skip first and last point)
            outlier_mask[0] = False
            outlier_mask[-1] = False
            
            for i in range(1, n - 1):
                if mad > 0:
                    if np.abs(vel_diff[i-1]) > threshold * mad or np.abs(vel_diff[i]) > threshold * mad:
                        outlier_mask[i] = True
            
            logger.info(f"Velocity Gradient Filter: {np.sum(outlier_mask)} Ausreißer (threshold={threshold}*MAD)")
        
        elif method == "median_diff":
            # Compare to local median
            outlier_mask = np.zeros(n, dtype=bool)
            half_window = window_size // 2
            
            for i in range(n):
                # Get window around point
                start = max(0, i - half_window)
                end = min(n, i + half_window + 1)
                
                window_vel = velocity[start:end]
                local_median = np.median(window_vel)
                local_mad = np.median(np.abs(window_vel - local_median))
                
                # Outlier if > threshold * MAD from local median
                if local_mad > 0:
                    if np.abs(velocity[i] - local_median) > threshold * local_mad:
                        outlier_mask[i] = True
            
            logger.info(f"Velocity Median Diff Filter: {np.sum(outlier_mask)} Ausreißer (window={window_size}, threshold={threshold})")
        
        else:
            raise ValueError(f"Unbekannte Methode: {method}")
        
        # Create filtered velocity (replace outliers with interpolation)
        filtered_velocity = velocity.copy()
        
        if np.any(outlier_mask):
            # Linear interpolation for outlier points
            valid_indices = np.where(~outlier_mask)[0]
            
            if len(valid_indices) > 1:
                filtered_velocity = np.interp(
                    np.arange(n),
                    valid_indices,
                    velocity[valid_indices]
                )
                logger.info(f"  → Interpoliert {np.sum(outlier_mask)} Ausreißer")
        
        return filtered_velocity, outlier_mask

# ============================================================================
# DATA ANALYSIS
# ============================================================================

@dataclass
class StatisticalSummary:
    """Statistical summary of measurement data."""
    column: str
    mean: float
    std: float
    min: float
    max: float
    median: float
    q25: float
    q75: float
    
    def __str__(self) -> str:
        return f"""
Statistical Summary for '{self.column}':
  Mean:       {self.mean:.6f}
  Std Dev:    {self.std:.6f}
  Min:        {self.min:.6f}
  Max:        {self.max:.6f}
  Median:     {self.median:.6f}
  Q25:        {self.q25:.6f}
  Q75:        {self.q75:.6f}
"""

class DataAnalyzer:
    """Analyze measurement data."""
    
    @staticmethod
    def get_statistics(df: pl.DataFrame, column: str) -> StatisticalSummary:
        """Calculate statistical summary."""
        values = df[column].to_numpy()
        
        return StatisticalSummary(
            column=column,
            mean=float(np.mean(values)),
            std=float(np.std(values)),
            min=float(np.min(values)),
            max=float(np.max(values)),
            median=float(np.median(values)),
            q25=float(np.percentile(values, 25)),
            q75=float(np.percentile(values, 75))
        )
    
    @staticmethod
    def get_settling_time(
        df: pl.DataFrame,
        column: str,
        tolerance_percent: float = 2.0
    ) -> Optional[float]:
        """
        Estimate settling time (when signal enters tolerance band).
        
        Args:
            df: Input DataFrame
            column: Column to analyze
            tolerance_percent: Tolerance band in percent
        
        Returns:
            Settling time in seconds, or None if not found
        """
        values = df[column].to_numpy()
        final_value = values[-1]
        tolerance_band = (final_value * tolerance_percent / 100.0)
        
        lower_bound = final_value - tolerance_band
        upper_bound = final_value + tolerance_band
        
        settled_indices = np.where(
            (values >= lower_bound) & (values <= upper_bound)
        )[0]
        
        if len(settled_indices) == 0:
            return None
        
        settling_idx = settled_indices[0]
        settling_time = df["timestamp_s"][settling_idx]
        
        return settling_time
    
    @staticmethod
    def get_rise_time(
        df: pl.DataFrame,
        column: str,
        start_percent: float = 10.0,
        end_percent: float = 90.0
    ) -> Optional[float]:
        """
        Calculate rise time (10% to 90% of final value).
        
        Args:
            df: Input DataFrame
            column: Column to analyze
            start_percent: Start threshold in percent
            end_percent: End threshold in percent
        
        Returns:
            Rise time in seconds, or None if not found
        """
        values = df[column].to_numpy()
        initial_value = values[0]
        final_value = values[-1]
        
        range_val = final_value - initial_value
        start_value = initial_value + (range_val * start_percent / 100.0)
        end_value = initial_value + (range_val * end_percent / 100.0)
        
        start_idx = np.where(values >= start_value)[0]
        end_idx = np.where(values >= end_value)[0]
        
        if len(start_idx) == 0 or len(end_idx) == 0:
            return None
        
        rise_time = df["timestamp_s"][end_idx[0]] - df["timestamp_s"][start_idx[0]]
        return rise_time
    
    @staticmethod
    def get_overshoot_percent(
        df: pl.DataFrame,
        column: str
    ) -> float:
        """
        Calculate overshoot as percentage of final value.
        
        Args:
            df: Input DataFrame
            column: Column to analyze
        
        Returns:
            Overshoot in percent
        """
        values = df[column].to_numpy()
        final_value = values[-1]
        max_value = np.max(values)
        
        if abs(final_value) < 1e-10:
            return 0.0
        
        overshoot = ((max_value - final_value) / abs(final_value)) * 100.0
        return max(0.0, overshoot)

# ============================================================================
# VISUALIZATION
# ============================================================================

class Plotter:
    """Generate plots from analysis data."""
    
    @staticmethod
    def plot_measurement(
        df: pl.DataFrame,
        output_path: Optional[Path] = None,
        title: str = "Measurement Data",
        show: bool = True,
        velocity_filter_method: Optional[str] = "gradient"
    ) -> None:
        """
        Generate 3-panel plot (Force, Cable Length, Cable Velocity) with optional filtering.
        
        Args:
            df: Input DataFrame
            output_path: Save location (if None, show interactive)
            title: Plot title
            show: Whether to show plot interactively
            velocity_filter_method: Filter method for velocity outliers
                ('zscore', 'iqr', 'gradient', 'median_diff', or None for no filtering)
        """
        time_s = df["timestamp_s"].to_numpy() - df["timestamp_s"][0]
        
        fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
        
        # Force
        axes[0].plot(time_s, df["force_N"].to_numpy(), 'b-', linewidth=1, label='Force')
        axes[0].set_ylabel("Force [N]", fontsize=11)
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        
        # Cable length
        axes[1].plot(time_s, df["cable_length_mm"].to_numpy(), 'r-', linewidth=1, label='Cable Length')
        axes[1].set_ylabel("Cable Length [mm]", fontsize=11)
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        
        # Cable velocity (robust calculation over window)
        smoother = SignalFilter()
        cable_vel = smoother.calculate_velocity_robust(
            df, 
            position_column="cable_length_mm",
            time_column="timestamp_s",
            window_samples=20,  # Adjust based on sample rate
            method="moving_window"  # Options: "moving_window", "savitzky_golay", "polyfit"
        )
        
        # Filter velocity outliers if requested
        cable_vel_filtered = cable_vel
        outlier_mask = None
        if velocity_filter_method:
            kwargs = {}
            if velocity_filter_method == "median_diff":
                kwargs = {"window_size": 5}
            elif velocity_filter_method in ["zscore", "iqr", "gradient"]:
                kwargs = {"threshold": 3.0 if velocity_filter_method != "iqr" else 1.5}
            
            cable_vel_filtered, outlier_mask = smoother.filter_velocity_outliers(
                cable_vel,
                method=velocity_filter_method,
                **kwargs
            )
        
        # Plot velocity with optional outlier highlighting
        axes[2].plot(time_s, cable_vel, 'b-', linewidth=0.8, alpha=0.5, label='Raw velocity')
        
        if velocity_filter_method and outlier_mask is not None:
            # Show outliers in red
            outlier_indices = np.where(outlier_mask)[0]
            if len(outlier_indices) > 0:
                axes[2].scatter(time_s[outlier_indices], cable_vel[outlier_indices],
                               color='red', s=30, marker='x', label=f'Outliers ({len(outlier_indices)})',
                               linewidths=2, zorder=5)
            
            # Plot filtered velocity
            axes[2].plot(time_s, cable_vel_filtered, 'g-', linewidth=1.5, 
                        label=f'Filtered velocity ({velocity_filter_method})', zorder=4)
        else:
            axes[2].plot(time_s, cable_vel_filtered, 'g-', linewidth=1, label='Cable Velocity')
        
        axes[2].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        axes[2].set_ylabel("Cable Velocity [mm/s]", fontsize=11)
        axes[2].set_xlabel("Time [s]", fontsize=11)
        axes[2].grid(True, alpha=0.3)
        axes[2].legend()
        
        fig.suptitle(title, fontsize=13, fontweight='bold')
        plt.tight_layout()
        
        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            logger.info(f"Plot gespeichert: {output_path}")
        else:
            if show:
                plt.show()
        
        plt.close()
    
    @staticmethod
    def plot_comparison(
        dataframes: List[Tuple[str, pl.DataFrame]],
        column: str = "force_N",
        output_path: Optional[Path] = None
    ) -> None:
        """
        Plot multiple measurements for comparison.
        
        Args:
            dataframes: List of (name, df) tuples
            column: Column to plot
            output_path: Save location
        """
        fig, ax = plt.subplots(figsize=(14, 6))
        
        for name, df in dataframes:
            time_s = df["timestamp_s"].to_numpy() - df["timestamp_s"][0]
            ax.plot(time_s, df[column].to_numpy(), linewidth=1.5, label=name)
        
        ax.set_xlabel("Time [s]", fontsize=11)
        ax.set_ylabel(f"{column}", fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        fig.suptitle(f"Comparison: {column}", fontsize=13, fontweight='bold')
        plt.tight_layout()
        
        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            logger.info(f"Plot gespeichert: {output_path}")
        else:
            plt.show()
        
        plt.close()
    
    @staticmethod
    def debug_velocity_calculation(
        df: pl.DataFrame,
        output_dir: Optional[Path] = None,
        window_samples: int = 20
    ) -> None:
        """
        Debug velocity calculation by visualizing all data points and time intervals.
        Includes velocity outlier filtering comparison.
        
        Args:
            df: Input DataFrame
            output_dir: Directory to save debug plots (if None, show interactive)
            window_samples: Window size for velocity calculation
        """
        time_s = df["timestamp_s"].to_numpy()
        cable_length = df["cable_length_mm"].to_numpy()
        n = len(df)
        
        # === DEBUG OUTPUTS ===
        logger.info("\n" + "="*70)
        logger.info("DEBUG: VELOCITY CALCULATION ANALYSIS")
        logger.info("="*70)
        
        logger.info(f"\n📊 Dataset Info:")
        logger.info(f"  Total samples: {n}")
        logger.info(f"  Time range: {time_s[0]:.6f}s - {time_s[-1]:.6f}s")
        logger.info(f"  Total duration: {time_s[-1] - time_s[0]:.6f}s")
        
        # Time delta analysis
        time_deltas = np.diff(time_s)
        logger.info(f"\n⏱️ Time Delta Analysis:")
        logger.info(f"  Min dt: {np.min(time_deltas):.9f}s")
        logger.info(f"  Max dt: {np.max(time_deltas):.9f}s")
        logger.info(f"  Mean dt: {np.mean(time_deltas):.9f}s")
        logger.info(f"  Std dt: {np.std(time_deltas):.9f}s")
        logger.info(f"  Sample rate (1/mean_dt): {1/np.mean(time_deltas):.0f}Hz")
        
        # Position delta analysis
        pos_deltas = np.diff(cable_length)
        logger.info(f"\n📍 Position Delta Analysis:")
        logger.info(f"  Min Δpos: {np.min(pos_deltas):.9f}mm")
        logger.info(f"  Max Δpos: {np.max(pos_deltas):.9f}mm")
        logger.info(f"  Mean Δpos: {np.mean(pos_deltas):.9f}mm")
        logger.info(f"  Std Δpos: {np.std(pos_deltas):.9f}mm")
        
        # Show first 20 samples
        logger.info(f"\n🔍 First 20 Time Intervals:")
        for i in range(min(20, len(time_deltas))):
            vel_simple = pos_deltas[i] / time_deltas[i] if time_deltas[i] > 0 else 0
            logger.info(f"  [{i:2d}] dt={time_deltas[i]:.9f}s | Δpos={pos_deltas[i]:+.6f}mm | v_simple={vel_simple:+.3f}mm/s")
        
        # Calculate velocity with different methods
        logger.info(f"\n📈 Velocity Calculation (window_samples={window_samples}):")
        
        smoother = SignalFilter()
        vel_moving = smoother.calculate_velocity_robust(
            df, 
            position_column="cable_length_mm",
            time_column="timestamp_s",
            window_samples=window_samples,
            method="moving_window"
        )
        
        logger.info(f"  Velocity Min: {np.min(vel_moving):.6f}mm/s")
        logger.info(f"  Velocity Max: {np.max(vel_moving):.6f}mm/s")
        logger.info(f"  Velocity Mean: {np.mean(vel_moving):.6f}mm/s")
        logger.info(f"  Velocity Std: {np.std(vel_moving):.6f}mm/s")
        logger.info(f"  Velocity Range: {np.max(vel_moving) - np.min(vel_moving):.6f}mm/s")
        
        # Show first 20 velocities
        logger.info(f"\n  First 20 Velocity Values:")
        for i in range(min(20, len(vel_moving))):
            logger.info(f"  [{i:2d}] v={vel_moving[i]:+.3f}mm/s")
        
        # === FILTER VELOCITY OUTLIERS ===
        logger.info(f"\n🔧 Velocity Outlier Filtering:")
        
        # Use top 3 filter methods (gradient, median_diff, zscore)
        filter_methods = {
            "gradient": {"threshold": 3.0},
            "median_diff": {"threshold": 2.5, "window_size": 5},
            "zscore": {"threshold": 3.0}
        }
        
        filtered_results = {}
        for method_name, kwargs in filter_methods.items():
            logger.info(f"\n  Method: {method_name}")
            filtered_vel, outlier_mask = smoother.filter_velocity_outliers(
                vel_moving,
                method=method_name,
                **kwargs
            )
            filtered_results[method_name] = {
                "filtered": filtered_vel,
                "outliers": outlier_mask,
                "kwargs": kwargs
            }
            outlier_count = np.sum(outlier_mask)
            logger.info(f"    Outliers: {outlier_count} | Min: {np.min(filtered_vel):.6f}mm/s | Max: {np.max(filtered_vel):.6f}mm/s | Mean: {np.mean(filtered_vel):.6f}mm/s")
        
        # === CREATE DEBUG PLOTS ===
        fig = plt.figure(figsize=(20, 14))
        
        # Plot 1: All raw time points
        ax1 = plt.subplot(3, 3, 1)
        ax1.scatter(range(n), time_s, alpha=0.6, s=10, label='Time values')
        ax1.plot(time_s, 'b-', linewidth=0.5, alpha=0.3)
        ax1.set_xlabel("Sample Index", fontsize=10)
        ax1.set_ylabel("Time [s]", fontsize=10)
        ax1.set_title("1. Raw Time Values (all points)", fontsize=11, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # Plot 2: Time deltas
        ax2 = plt.subplot(3, 3, 2)
        ax2.scatter(range(len(time_deltas)), time_deltas * 1e6, alpha=0.6, s=10, color='orange')
        ax2.axhline(y=np.mean(time_deltas)*1e6, color='r', linestyle='--', label=f'Mean={np.mean(time_deltas)*1e6:.3f}µs')
        ax2.set_xlabel("Sample Index", fontsize=10)
        ax2.set_ylabel("Δt [µs]", fontsize=10)
        ax2.set_title("2. Time Deltas (dt between samples)", fontsize=11, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        # Plot 3: Cable length (raw)
        ax3 = plt.subplot(3, 3, 3)
        ax3.scatter(time_s, cable_length, alpha=0.6, s=10, color='green')
        ax3.plot(time_s, cable_length, 'g-', linewidth=0.5, alpha=0.3)
        ax3.set_xlabel("Time [s]", fontsize=10)
        ax3.set_ylabel("Cable Length [mm]", fontsize=10)
        ax3.set_title("3. Raw Cable Length Data (all points)", fontsize=11, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        
        # Plot 4: Position deltas
        ax4 = plt.subplot(3, 3, 4)
        ax4.scatter(range(len(pos_deltas)), pos_deltas, alpha=0.6, s=10, color='purple')
        ax4.axhline(y=np.mean(pos_deltas), color='r', linestyle='--', label=f'Mean={np.mean(pos_deltas):.6f}mm')
        ax4.set_xlabel("Sample Index", fontsize=10)
        ax4.set_ylabel("Δpos [mm]", fontsize=10)
        ax4.set_title("4. Position Deltas (Δpos between samples)", fontsize=11, fontweight='bold')
        ax4.grid(True, alpha=0.3)
        ax4.legend()
        
        # Plot 5: Simple velocity (point-to-point)
        ax5 = plt.subplot(3, 3, 5)
        vel_simple = pos_deltas / time_deltas
        ax5.scatter(time_s[:-1], vel_simple, alpha=0.4, s=5, color='red', label='Point-to-point velocity')
        ax5.plot(time_s[:-1], vel_simple, 'r-', linewidth=0.5, alpha=0.2)
        ax5.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        ax5.set_xlabel("Time [s]", fontsize=10)
        ax5.set_ylabel("Velocity [mm/s]", fontsize=10)
        ax5.set_title(f"5. Simple Velocity (Δpos/Δt) - NOISY", fontsize=11, fontweight='bold')
        ax5.grid(True, alpha=0.3)
        ax5.legend()
        
        # Plot 6: Robust velocity (windowed)
        ax6 = plt.subplot(3, 3, 6)
        ax6.scatter(time_s, vel_moving, alpha=0.6, s=10, color='blue', label=f'Windowed velocity (w={window_samples})')
        ax6.plot(time_s, vel_moving, 'b-', linewidth=1, alpha=0.5)
        ax6.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        ax6.set_xlabel("Time [s]", fontsize=10)
        ax6.set_ylabel("Velocity [mm/s]", fontsize=10)
        ax6.set_title(f"6. Robust Velocity (moving window, w={window_samples})", fontsize=11, fontweight='bold')
        ax6.grid(True, alpha=0.3)
        ax6.legend()
        
        # Plot 7-9: Filtered velocities for top 3 methods
        colors = {'gradient': 'orange', 'median_diff': 'cyan', 'zscore': 'purple'}
        for plot_idx, (method_name, result) in enumerate(filtered_results.items(), 1):
            ax = plt.subplot(3, 3, 6 + plot_idx)
            
            filtered_vel = result["filtered"]
            outlier_mask = result["outliers"]
            
            # Plot original with outliers marked
            ax.scatter(time_s, vel_moving, alpha=0.3, s=5, color='lightgray', label='Original')
            
            # Plot outliers in red
            outlier_indices = np.where(outlier_mask)[0]
            if len(outlier_indices) > 0:
                ax.scatter(time_s[outlier_indices], vel_moving[outlier_indices], 
                          alpha=0.8, s=20, color='red', marker='x', label=f'Outliers ({len(outlier_indices)})', linewidths=2)
            
            # Plot filtered
            ax.plot(time_s, filtered_vel, linewidth=1.5, color=colors.get(method_name, 'black'), 
                   label=f'Filtered ({method_name})', alpha=0.8)
            
            ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
            ax.set_xlabel("Time [s]", fontsize=10)
            ax.set_ylabel("Velocity [mm/s]", fontsize=10)
            
            kwargs_str = ", ".join([f"{k}={v}" for k, v in result["kwargs"].items()])
            ax.set_title(f"{6+plot_idx}. {method_name.upper()}\n({kwargs_str})", fontsize=10, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8)
        
        fig.suptitle("DEBUG: Velocity Calculation & Outlier Filtering - All Data Points", fontsize=14, fontweight='bold')
        plt.tight_layout()
        
        if output_dir:
            output_dir = Path(output_dir)
            output_dir.mkdir(parents=True, exist_ok=True)
            output_path = output_dir / "debug_velocity_analysis.png"
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            logger.info(f"\n✅ Debug plot gespeichert: {output_path}")
        else:
            plt.show()
        
        plt.close()
        logger.info("="*70 + "\n")

# ============================================================================
# MAIN WORKFLOW
# ============================================================================

def main():
    """Example workflow for data analysis."""
    setup_logging()
    
    logger.info("=" * 70)
    logger.info("STEP RESPONSE DATENANALYSE")
    logger.info("=" * 70 + "\n")
    
    # Load test data for specific force (10.0 N)
    loader = DataLoader()
    all_data = loader.load_tests_by_force(force_N=10.0)
    
    if not all_data:
        logger.error("Keine Daten gefunden!")
        return
    
    logger.info(f"\n✅ {len(all_data)} Tests geladen\n")

    for name, df in all_data:
        logger.info(f"Analysiere: {name}")
        logger.info(f"Originale Größe: {len(df)} Samples\n")
        trimmer = DataTrimmier()
        windowed_df = trimmer.extract_window(df, skip_start_samples=100, num_samples_to_keep=300)
        plotter = Plotter()
        plotter.plot_measurement(windowed_df, title=f"Analysierte Messreihe: {name}", show=True)


    # # Analyze first test
    # name, df = all_data[0]
    # logger.info(f"Analysiere: {name}")
    # logger.info(f"Originale Größe: {len(df)} Samples\n")
    
    # # Trim data (remove first 0.5s and last 1.0s)
    # trimmer = DataTrimmier()
    # trimmed_df = trimmer.trim_by_time(df, trim_start_s=0.5, trim_end_s=1.0)
    
    # # Filter outliers using multiple methods
    # detector = OutlierDetector()
    
    # logger.info("\n--- Ausreißer-Filterung ---")
    # cleaned_zscore, _ = detector.zscore_filter(trimmed_df, column="force_N", threshold=3.0)
    # cleaned_iqr, _ = detector.iqr_filter(cleaned_zscore, column="cable_length_mm", k=1.5)
    
    # # Apply smoothing
    # logger.info("\n--- Glättung ---")
    # smoother = SignalFilter()
    # smoothed_df = smoother.savitzky_golay_filter(
    #     cleaned_iqr,
    #     column="force_N",
    #     window_length=11,
    #     polyorder=2
    # )
    
    # # Analyze
    # logger.info("\n--- Statistik ---")
    # analyzer = DataAnalyzer()
    
    # force_stats = analyzer.get_statistics(smoothed_df, "force_N")
    # cable_stats = analyzer.get_statistics(smoothed_df, "cable_length_mm")
    
    # logger.info(force_stats)
    # logger.info(cable_stats)
    
    # Performance metrics
    # logger.info("\n--- Leistungsmetriken ---")
    # rise_time = analyzer.get_rise_time(smoothed_df, "force_N")
    # settling_time = analyzer.get_settling_time(smoothed_df, "force_N", tolerance_percent=2.0)
    # overshoot = analyzer.get_overshoot_percent(smoothed_df, "force_N")
    
    # logger.info(f"Anstiegszeit (10%-90%): {rise_time:.3f}s" if rise_time else "N/A")
    # logger.info(f"Einschwingzeit (±2%):   {settling_time:.3f}s" if settling_time else "N/A")
    # logger.info(f"Überschwing:            {overshoot:.2f}%")
    
    # Generate plots
    # logger.info("\n--- Visualisierung ---")
    # plotter = Plotter()
    # plotter.plot_measurement(smoothed_df, title=f"Analysierte Messreihe: {name}")

if __name__ == "__main__":
    main()
