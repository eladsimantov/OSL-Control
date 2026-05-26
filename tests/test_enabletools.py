#!/usr/bin/env python3
import os
import sys
import unittest
import numpy as np

# Path setup to include project root
tests_path = os.path.dirname(os.path.abspath(__file__))
project_path = os.path.join(tests_path, "..")
sys.path.insert(0, project_path)

from src.enabletools import (
    quat_to_euler, 
    rotate_vector, 
    JointVelocityFilter, 
    ScalarKalmanFilter, 
    impedance_control,
    OnlinePCA,
    cvp_controller
)

class TestEnableTools(unittest.TestCase):
    
    def test_quat_to_euler_identity(self):
        # Identity quaternion (no rotation)
        yaw, pitch, roll = quat_to_euler([1.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(yaw, 0.0)
        self.assertAlmostEqual(pitch, 0.0)
        self.assertAlmostEqual(roll, 0.0)

    def test_quat_to_euler_known_rotation(self):
        # 90 degrees rotation around Z axis (yaw = 90, pitch = 0, roll = 0)
        # q = cos(45) + sin(45)*z = 0.70710678 + 0.70710678*z
        yaw, pitch, roll = quat_to_euler([0.70710678, 0.0, 0.0, 0.70710678])
        self.assertAlmostEqual(yaw, 90.0, places=4)
        self.assertAlmostEqual(pitch, 0.0, places=4)
        self.assertAlmostEqual(roll, 0.0, places=4)

    def test_rotate_vector(self):
        # Rotate unit X vector [1, 0, 0] by 90 degrees around Z axis -> should get [0, 1, 0]
        v = [1.0, 0.0, 0.0]
        q = [0.70710678, 0.0, 0.0, 0.70710678]
        v_rot = rotate_vector(v, q)
        self.assertAlmostEqual(v_rot[0], 0.0, places=4)
        self.assertAlmostEqual(v_rot[1], 1.0, places=4)
        self.assertAlmostEqual(v_rot[2], 0.0, places=4)

    def test_scalar_kalman_filter(self):
        # Filter should smooth data
        f = ScalarKalmanFilter(initial_value=0.0, process_noise=1.0, measurement_noise=1.0)
        # Step input
        val = f.update(10.0)
        self.assertTrue(0.0 < val < 10.0)  # Should start converging towards 10.0

    def test_joint_velocity_filter(self):
        # Verify kinematics filter estimates velocity
        dt = 0.01
        f = JointVelocityFilter(dt=dt, process_noise=100.0, measurement_noise=0.01)
        # Simulate constant velocity movement (10 units per second)
        pos = 0.0
        for _ in range(100):
            pos += 10.0 * dt
            pos_f, vel_f = f.update(pos)
        
        # Velocity estimate should be positive and close to 10.0
        self.assertTrue(vel_f > 0.0)
        self.assertAlmostEqual(vel_f, 10.0, delta=1.0)

    def test_impedance_control(self):
        # tau = - kp * (q - q_eq) - kd * dq
        # - 100 * (1.0 - 0.0) - 10 * 0 = -100
        tau = impedance_control(q=1.0, dq=0.0, q_eq=0.0, kp=100.0, kd=10.0)
        self.assertEqual(tau, -100.0)

    def test_online_pca_and_cvp(self):
        pca = OnlinePCA(dim=3, update_interval=1)
        
        # Feed points that lie exactly on a plane: z = x + y -> x + y - z = 0
        # Normal vector should be parallel to [1, 1, -1] / sqrt(3) = [0.577, 0.577, -0.577]
        # In our cvp form: thigh = x, shank = y, foot = z
        # X_s = mu_s + (1/v_3s) * ((X_t - mu_t)*v_3t + (X_f - mu_f)*v_3f)
        # Here: X_s = X_t + X_f -> mu_s = 0, mu_t = 0, mu_f = 0
        # Normal vector components: v_3t = 1, v_3s = -1, v_3f = 1
        
        # Generate data points with zero mean
        np.random.seed(42)
        for _ in range(100):
            t = np.random.uniform(-1, 1)
            f = np.random.uniform(-1, 1)
            s = t + f
            pca.update(np.array([t, s, f]))
            
        V, mu = pca.components, pca.mean
        
        # Test CVP prediction on a new point: t=0.5, f=0.3 -> s should be 0.8
        s_pred = cvp_controller(0.5, 0.3, V, mu)
        self.assertAlmostEqual(s_pred, 0.8, places=4)

if __name__ == "__main__":
    unittest.main()
