import numpy as np
from filterpy.kalman import KalmanFilter

class JointVelocityFilter:
    """Uses a Kinematic Kalman Filter to estimate joint position and velocity.
    
    This avoids the noise magnification of raw numerical differentiation while
    introducing minimal lag compared to traditional low-pass filtering.
    """
    def __init__(self, dt: float, process_noise: float = 100.0, measurement_noise: float = 0.01):
        """
        Args:
            dt (float): Time step in seconds between updates.
            process_noise (float): Process noise covariance multiplier (model uncertainty).
            measurement_noise (float): Measurement noise covariance (sensor uncertainty).
        """
        self.kf = KalmanFilter(dim_x=2, dim_z=1)
        self.kf.x = np.array([0., 0.])  # State: [position, velocity]
        
        # State transition matrix
        self.kf.F = np.array([[1., dt],
                              [0., 1.]])
        
        # Measurement function
        self.kf.H = np.array([[1., 0.]])
        
        # Measurement uncertainty
        self.kf.R *= measurement_noise
        
        # Process uncertainty
        self.kf.Q = np.array([[0.25 * (dt**4), 0.5 * (dt**3)],
                              [0.5 * (dt**3), dt**2]]) * process_noise
                              
        # Covariance matrix
        self.kf.P *= 1.0

    def update(self, pos_measured: float) -> tuple[float, float]:
        """Predicts and updates the filter state.
        
        Args:
            pos_measured (float): The raw measured joint position.
            
        Returns:
            tuple: (position_filtered, velocity_filtered)
        """
        self.kf.predict()
        self.kf.update(pos_measured)
        return float(self.kf.x[0]), float(self.kf.x[1])


class ScalarKalmanFilter:
    """A simple 1D Kalman filter for smoothing noisy scalar inputs (e.g., loadcell forces)
    in real time without the lag of a large moving average window.
    """
    def __init__(self, initial_value: float = 0.0, process_noise: float = 0.1, measurement_noise: float = 10.0):
        """
        Args:
            initial_value (float): Starting value for the filter.
            process_noise (float): Process covariance (how fast the system changes).
            measurement_noise (float): Measurement covariance (how noisy the sensor is).
        """
        self.kf = KalmanFilter(dim_x=1, dim_z=1)
        self.kf.x = np.array([initial_value])
        self.kf.F = np.array([[1.]])
        self.kf.H = np.array([[1.]])
        self.kf.Q *= process_noise
        self.kf.R *= measurement_noise
        self.kf.P *= 1.0

    def update(self, val_measured: float) -> float:
        """Predicts and updates the filter state.
        
        Args:
            val_measured (float): Raw scalar measurement.
            
        Returns:
            float: Smoothed/filtered value.
        """
        self.kf.predict()
        self.kf.update(val_measured)
        return float(self.kf.x[0])


class OnlinePCA:
    """Computes Principal Component Analysis (PCA) online sample-by-sample.
    
    Uses Welford's algorithm for numerically stable running covariance estimation
    and performs eigendecomposition on the covariance matrix.
    
    Highly optimized for low-dimensional systems (e.g. 3D limb elevation angles).
    """
    def __init__(self, dim: int = 3, update_interval: int = 1):
        """
        Args:
            dim (int): Dimension of the input data (default 3 for thigh-shank-foot).
            update_interval (int): Perform eigendecomposition every N samples.
        """
        self.dim = dim
        self.update_interval = update_interval
        self.n = 0
        self.mean = np.zeros(dim)
        self.S = np.zeros((dim, dim))  # Sum of square differences (Welford's matrix)
        self.components = np.eye(dim)  # Row vectors represent principal components (eigenvectors)
        self.explained_variance = np.zeros(dim)

    def update(self, x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Update running statistics with a new sample and optionally recompute PCA.
        
        Args:
            x (np.ndarray): 1D array of shape (dim,) representing the new sample.
            
        Returns:
            tuple: (eigenvectors_as_columns, mean)
                eigenvectors_as_columns is a (dim, dim) matrix where columns are eigenvectors,
                sorted by descending variance (column 0 is PC1, column -1 is the normal vector).
        """
        self.n += 1
        x = np.asarray(x, dtype=float)
        
        # Welford's algorithm for running covariance
        delta = x - self.mean
        self.mean += delta / self.n
        
        if self.n > 1:
            self.S += np.outer(delta, x - self.mean)
            
        # Periodically compute eigendecomposition
        if self.n > 1 and (self.n % self.update_interval == 0):
            cov = self.S / (self.n - 1)
            # eigh is highly optimized for symmetric matrices like covariance
            eigenvalues, eigenvectors = np.linalg.eigh(cov)
            
            # Sort in descending order of eigenvalues (eigenvalues returned are ascending by default)
            idx = np.argsort(eigenvalues)[::-1]
            self.explained_variance = eigenvalues[idx]
            # Store eigenvectors as columns of self.components (matching column convention)
            self.components = eigenvectors[:, idx]
            
        return self.components, self.mean
