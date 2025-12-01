import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RobotArm:
    def __init__(self, link_length=1.0):
        """
        Initialize the robot arm with a fixed link length.
        
        Args:
            link_length (float): The length of each link 'L'. Default is 1.0m.
        """
        self.L = link_length

    def dh_matrix(self, theta, d, a, alpha):
        """
        Create the Denavit-Hartenberg transformation matrix.
        
        Args:
            theta (float): Joint angle (rotation about Z_i-1) in radians.
            d (float): Link offset (translation along Z_i-1).
            a (float): Link length (translation along X_i).
            alpha (float): Link twist (rotation about X_i) in radians.
            
        Returns:
            np.ndarray: 4x4 homogeneous transformation matrix.
        """
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_alpha = np.cos(alpha)
        s_alpha = np.sin(alpha)

        return np.array([
            [c_theta, -s_theta * c_alpha,  s_theta * s_alpha, a * c_theta],
            [s_theta,  c_theta * c_alpha, -c_theta * s_alpha, a * s_theta],
            [0,        s_alpha,            c_alpha,           d],
            [0,        0,                  0,                 1]
        ])

    def forward_kinematics(self, joint_angles):
        """
        Calculate the forward kinematics of the robot arm.
        
        Args:
            joint_angles (list or np.array): A list of 4 joint angles [j1, j2, j3, j4] in radians.
            
        Returns:
            np.ndarray: The [x, y, z] coordinates of the end-effector.
        """
        if len(joint_angles) != 4:
            raise ValueError("Expected 4 joint angles.")

        j1, j2, j3, j4 = joint_angles
        L = self.L

        
        dh_params = [
            (j1, 0, L, np.pi/2),  # Joint 1 to 2
            (j2, 0, L, np.pi/2),  # Joint 2 to 3
            (j3, 0, L, np.pi/2),  # Joint 3 to 4
            (j4, 0, L, 0)         # Joint 4 to End Effector
        ]

        # Initialize transformation matrix as Identity
        T = np.eye(4)

        # Multiply transformation matrices
        for params in dh_params:
            T_i = self.dh_matrix(*params)
            T = np.dot(T, T_i)

        # The position is the translation component (first 3 rows of the 4th column)
        position = T[:3, 3]
        return position

    def calculate_joint_positions(self, joint_angles):
        """
        Calculate the positions of all joints for visualization.
        
        Args:
            joint_angles (list): List of 4 joint angles.
            
        Returns:
            np.ndarray: Array of shape (5, 3) containing [x, y, z] for Base, J1, J2, J3, End-Effector.
        """
        if len(joint_angles) != 4:
            raise ValueError("Expected 4 joint angles.")

        j1, j2, j3, j4 = joint_angles
        L = self.L
        
        dh_params = [
            (j1, 0, L, np.pi/2),
            (j2, 0, L, np.pi/2),
            (j3, 0, L, np.pi/2),
            (j4, 0, L, 0)
        ]

        T = np.eye(4)
        positions = [T[:3, 3]] # Start with base at 0,0,0
        
        for params in dh_params:
            T_i = self.dh_matrix(*params)
            T = np.dot(T, T_i)
            positions.append(T[:3, 3])
            
        return np.array(positions)

def main():
    # Initialize robot with link length 1m
    robot = RobotArm(link_length=1.0)
    
    print("Forward Kinematics Calculator")
    print("Enter 4 joint angles in radians separated by spaces (e.g., '0 1.57 0 0').")
    print("Type 'exit' or 'quit' to stop.")

    while True:
        try:
            user_input = input("\nEnter joint angles (j1 j2 j3 j4): ")
            
            if user_input.lower() in ['exit', 'quit']:
                break
            
            # Parse input
            angles = [float(x) for x in user_input.strip().split()]
            
            if len(angles) != 4:
                print("Error: Please enter exactly 4 angles.")
                continue
                
            # Calculate position
            pos = robot.forward_kinematics(angles)
            print(f"End Effector Position (x, y, z): {pos}")
            
            # Visualization
            print("Displaying plot... (Close the plot window to continue)")
            joint_positions = robot.calculate_joint_positions(angles)
            
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            
            xs = joint_positions[:, 0]
            ys = joint_positions[:, 1]
            zs = joint_positions[:, 2]
            
            ax.plot(xs, ys, zs, '-o', label='Robot Arm', linewidth=2, markersize=6)
            
            # Plot origin
            ax.scatter([0], [0], [0], color='r', s=50, label='Base')
            
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title(f'Robot Arm Configuration\nAngles: {angles}')
            
            # Set consistent axis limits
            limit = 4.0
            ax.set_xlim(-limit, limit)
            ax.set_ylim(-limit, limit)
            ax.set_zlim(-limit, limit)
            
            ax.legend()
            plt.show()
            
        except ValueError:
            print("Error: Invalid input. Please enter numbers only.")
        except Exception as e:
            print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
