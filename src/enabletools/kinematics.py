from pyquaternion import Quaternion
import math

def quat_to_euler(q_wxyz, degrees: bool = True) -> tuple[float, float, float]:
    """Convert wxyz quaternion to Tait-Bryan intrinsic Z-Y'-X'' euler angles (yaw, pitch, roll).
    
    This matches the BNO055 wxyz tuple ordering directly and avoids the 
    known firmware euler conversion bug on the BNO055 chip.
    
    Args:
        q_wxyz (tuple/list): 4-element quaternion in (w, x, y, z) order.
        degrees (bool): If True, returns angles in degrees. Defaults to True.
        
    Returns:
        tuple: (yaw, pitch, roll) in degrees or radians.
    """
    q = Quaternion(q_wxyz)
    yaw, pitch, roll = q.yaw_pitch_roll
    if degrees:
        return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)
    return yaw, pitch, roll

def rotate_vector(v, q_wxyz) -> list[float]:
    """Rotate a 3D vector by a wxyz quaternion.
    
    Useful for rotating loadcell force readings from the sensor frame into the bone/global frame.
    
    Args:
        v (list/tuple): 3-element vector to rotate.
        q_wxyz (list/tuple): 4-element quaternion in (w, x, y, z) order.
        
    Returns:
        list: 3-element rotated vector.
    """
    q = Quaternion(q_wxyz)
    return list(q.rotate(v))
