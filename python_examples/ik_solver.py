
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
    # 3-DOF Robot Configuration (matching the web simulator)
    dh_3dof = [
        (0,  np.pi/2, 15, 0), # Base
        (30, 0,        0, 0), # Shoulder
        (25, 0,        0, 0)  # Elbow
    ]
    
    robot = KinematicChain(dh_3dof)
    
    # --- SIMULATION DATA ---
    # Define a sequence of joint angles (a trajectory)
    # Each row is a waypoint: [J1, J2, J3]
    trajectory_angles = [
        [0, 0, 0],        # Home
        [45, -30, 20],    # Point A
        [90, 0, 45],      # Point B
        [-45, 30, -20],   # Point C
        [0, 0, 0]         # Return Home
    ]
    
    # Calculate XYZ positions for each waypoint
    positions = []
    for angles in trajectory_angles:
        pos = robot.forward_kinematics(angles)
        positions.append(pos)
        print(f"Angles: {angles}° -> XYZ: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] cm")
    
    positions = np.array(positions)

    # --- 3D PLOTTING WITH MATPLOTLIB ---
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the trajectory line
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
            marker='o', markersize=8, color='#38bdf8', linewidth=2.5, label='End-Effector Trajectory')

    # Add labels to the waypoints
    for i, (x, y, z) in enumerate(positions):
        ax.text(x, y, z, f"  P{i}", color='white', fontsize=10)

    # Draw the ground plane (Z=0) to visualize floor collisions
    xx, yy = np.meshgrid(np.linspace(-50, 50, 10), np.linspace(-50, 50, 10))
    zz = np.zeros_like(xx)
    ax.plot_surface(xx, yy, zz, alpha=0.2, color='gray')

    # Styling
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_zlabel('Z (cm)')
    ax.set_title('RoboControl 3D — Trajectory Simulation')
    
    # Set background color to match your web app (Dark Mode)
    fig.patch.set_facecolor('#0f172a')
    ax.set_facecolor('#0f172a')
    ax.xaxis.label.set_color('white')
    ax.yaxis.label.set_color('white')
    ax.zaxis.label.set_color('white')
    ax.title.set_color('white')
    ax.tick_params(colors='white')
    ax.legend()

    # Ensure axes are equal
    ax.set_box_aspect([1, 1, 1]) 

    plt.show()