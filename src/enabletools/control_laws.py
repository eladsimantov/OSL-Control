import numpy as np

def impedance_control(q: float, dq: float, q_eq: float, kp: float, kd: float) -> float:
    """Calculates active spring-damper virtual joint impedance command.
    
    Formula:
        tau = - kp * (q - q_eq) - kd * dq
        
    Args:
        q (float): Current joint position (e.g., in radians or degrees).
        dq (float): Current joint velocity (matching position units per second).
        q_eq (float): Equilibrium target joint position.
        kp (float): Proportional stiffness gain (torque / unit position).
        kd (float): Derivative damping gain (torque / unit velocity).
        
    Returns:
        float: Calculated target impedance torque command.
    """
    return - kp * (q - q_eq) - kd * dq

def cvp_controller(X_t: float, X_f: float, V: np.ndarray, mu: np.ndarray) -> float:
    """Calculate the shank elevation variable based on a covariation plane (CVP).

    Args:
        X_t (float): The thigh elevation space variable.
        X_f (float): The foot elevation space variable.
        V (np.ndarray): The Principal Component vectors in matrix form (eigenvectors as columns).
        mu (np.ndarray): The mean values of the thigh, shank and foot variables.
        
    Returns:
        X_s (float): The shank elevation space variable in the sagittal plane.
    
    Formula:
        v_3t*(X_t - mu_t) + v_3s*(X_s - mu_s) + v_3f*(X_f - mu_f) = 0
        X_s = mu_s - (1/v_3s) * ((X_t - mu_t)*v_3t + (X_f - mu_f)*v_3f)
    """
    X_s = mu[1] - (1/V[1, 2]) * ( (X_t - mu[0])*V[0, 2] + (X_f - mu[2])*V[2, 2] )
    return X_s
    
def estimate_T12(xt, xf, V, mu):
    """Calculate the instantaneous estimate of the PC scores given the thigh and foot 
    elevation space variables, the PC vector matrix and means.

    Args: 
        xt (float): The thigh elevation space variable.
        xf (float): The foot elevation space variable.
        V (np.ndarray): The Principal Component vectors in matrix form (eigenvectors as columns).
        mu (np.ndarray): The mean values of the thigh, shank and foot variables.
        
    Returns: 
        t1 (float): The first PC score estimate.
        t2 (float): The second PC score estimate.

    """
    A = V[[0,2], :2]
    x = np.array([xt - mu[0], xf - mu[2]])
    T12 = np.linalg.solve(A, x)
    return T12[0], T12[1]

class BaseController:
    """Base class for modular controllers."""
    def update(self, motor, thigh_imu, foot_imu, loadcell, t: float, state_time: float) -> None:
        raise NotImplementedError("Subclasses must implement update().")

class HoldImpedanceController(BaseController):
    """Holds a target position in degrees using virtual impedance control."""
    def __init__(self, kp: float = 0.02, kd: float = 0.0006, deg_eq: float = 0.0):
        self.kp = kp
        self.kd = kd
        self.deg_eq = deg_eq

    def update(self, motor, thigh_imu, foot_imu, loadcell, t: float, state_time: float) -> None:
        motor.set_impedance(kp=self.kp, kd=self.kd, deg_eq=self.deg_eq)

class SwingTrajectoryController(BaseController):
    """Interpolates the knee angle through a swing flexion trajectory using impedance control."""
    def __init__(self, kp: float = 0.02, kd: float = 0.0006, stance_angle: float = 5.0, 
                 peak_flexion_angle: float = 40.0, peak_flexion_time: float = 0.25, max_swing_time: float = 0.55):
        self.kp = kp
        self.kd = kd
        self.stance_angle = stance_angle
        self.peak_flexion_angle = peak_flexion_angle
        self.peak_flexion_time = peak_flexion_time
        self.max_swing_time = max_swing_time

    def update(self, motor, thigh_imu, foot_imu, loadcell, t: float, state_time: float) -> None:
        # Interpolate the equilibrium angle
        if state_time < self.peak_flexion_time:
            # Stance angle to peak flexion
            alpha = state_time / self.peak_flexion_time
            deg_eq = self.stance_angle + alpha * (self.peak_flexion_angle - self.stance_angle)
        else:
            # Peak flexion back to 0 (full extension)
            time_left = self.max_swing_time - self.peak_flexion_time
            if time_left > 0:
                alpha = (state_time - self.peak_flexion_time) / time_left
                deg_eq = self.peak_flexion_angle - alpha * self.peak_flexion_angle
                deg_eq = max(0.0, deg_eq)
            else:
                deg_eq = 0.0
        
        motor.set_impedance(kp=self.kp, kd=self.kd, deg_eq=deg_eq)

class CVPController(BaseController):
    """Calculates shank elevation using the covariation plane, and sets knee equilibrium angle in real-time."""
    def __init__(self, kp: float = 0.02, kd: float = 0.0006, V: np.ndarray = None, mu: np.ndarray = None, 
                 offset_deg: float = 0.0, thigh_imu_axis: str = 'y', foot_imu_axis: str = 'y'):
        self.kp = kp
        self.kd = kd
        self.offset_deg = offset_deg
        self.thigh_imu_axis = thigh_imu_axis
        self.foot_imu_axis = foot_imu_axis
        
        # Set default/representative V and mu if not provided
        if V is None:
            # Let's use a plane where z = x + y -> x + y - z = 0
            # Normal vector: [1, 1, -1] / sqrt(3)
            # Principal components: V[:, 2] is normal vector
            self.V = np.zeros((3, 3))
            self.V[:, 2] = np.array([1.0, -1.0, 1.0]) / np.sqrt(3.0)
        else:
            self.V = V
            
        if mu is None:
            self.mu = np.array([0.0, 0.0, 0.0])
        else:
            self.mu = mu

    def _get_imu_angle(self, imu, axis: str) -> float:
        """Helper to get angle from IMU based on selected axis."""
        if axis == 'x':
            return imu.euler_x
        elif axis == 'y':
            return imu.euler_y
        elif axis == 'z':
            return imu.euler_z
        return imu.euler_y

    def update(self, motor, thigh_imu, foot_imu, loadcell, t: float, state_time: float) -> None:
        # Read elevation angles from IMUs in degrees
        X_t = self._get_imu_angle(thigh_imu, self.thigh_imu_axis)
        X_f = self._get_imu_angle(foot_imu, self.foot_imu_axis)
        
        # Compute shank elevation using CVP
        X_s = cvp_controller(X_t, X_f, self.V, self.mu)
        
        # Knee joint angle is difference between thigh and shank elevation angles
        # plus the user-specified real-time offset
        deg_eq = X_t - X_s + self.offset_deg
        
        # Apply impedance control
        motor.set_impedance(kp=self.kp, kd=self.kd, deg_eq=deg_eq)
