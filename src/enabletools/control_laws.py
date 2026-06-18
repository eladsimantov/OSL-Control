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
