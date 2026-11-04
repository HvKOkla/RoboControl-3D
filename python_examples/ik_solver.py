"""
RoboControl 3D — Python Kinematics Example
This script demonstrates the same Forward and Inverse Kinematics 
used in the web-based simulator, using NumPy.
"""

import numpy as np

class KinematicChain:
    def __init__(self, dh_params):
        """
        DH params format: list of tuples (a, alpha, d, theta_offset)
        Angles are in radians.
        """
        self.dh_params = dh_params

    def get_transform_matrix(self, a, alpha, d, theta):
        """Standard Denavit-Hartenberg transformation matrix."""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,      sa,     ca,    d],
            [0,       0,      0,    1]
        ])

    def forward_kinematics(self, joint_angles_deg):
        """Computes the end-effector position from joint angles."""
        T = np.eye(4)
        angles_rad = np.radians(joint_angles_deg)
        
        for i, (a, alpha, d, theta_offset) in enumerate(self.dh_params):
            theta = angles_rad[i] + theta_offset
            T = T @ self.get_transform_matrix(a, alpha, d, theta)
            
        return T[:3, 3] # Return XYZ position

if __name__ == "__main__":
   
    dh_3dof = [
        (0,  np.pi/2, 15, 0), # Base
        (30, 0,        0, 0), # Shoulder
        (25, 0,        0, 0)  # Elbow
    ]
    
    robot = KinematicChain(dh_3dof)
    
   
    test_angles = [45, -30, 20]
    pos = robot.forward_kinematics(test_angles)
    
    print(f"Joint Angles: {test_angles}°")
    print(f"End-Effector Position (X, Y, Z): {pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f} cm")