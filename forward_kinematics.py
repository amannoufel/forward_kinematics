import numpy as np

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

        # DH Parameters based on the description:
        # "The axis of each joint is perpendicular to the previous joint."
        # "The lengths of the links - 'L' is 1m each."
        
        # Assumptions made from diagram:
        # Frame 0: Base
        # Frame 1: After J1 rotation, translated by L along link 1. Axis Z1 perpendicular to Z0.
        # Frame 2: After J2 rotation, translated by L along link 2. Axis Z2 perpendicular to Z1.
        # Frame 3: After J3 rotation, translated by L along link 3. Axis Z3 perpendicular to Z2.
        # Frame 4: After J4 rotation, translated by L along link 4. End effector.
        
        # DH Table (theta, d, a, alpha)
        # Note: We convert angles to radians if they aren't already, but input is assumed radians.
        # We use alternating 90 and -90 degrees for alpha to keep the arm somewhat "upright" in the diagrammatic sense,
        # ensuring Z axes are perpendicular.
        
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

def main():
    # Example usage
    try:
        # Initialize robot with link length 1m
        robot = RobotArm(link_length=1.0)

        # Test Case 1: All zeros
        # If all angles are zero, the arm should be extended.
        angles_1 = [0, 0, 0, 0]
        pos_1 = robot.forward_kinematics(angles_1)
        print(f"Angles: {angles_1} -> End Effector Position: {pos_1}")

        # Test Case 2: 90 degrees on first joint
        angles_2 = [np.pi/2, 0, 0, 0]
        pos_2 = robot.forward_kinematics(angles_2)
        print(f"Angles: [90 deg, 0, 0, 0] -> End Effector Position: {pos_2}")
        
        # Test Case 3: Random configuration
        angles_3 = [0.1, 0.2, 0.3, 0.4]
        pos_3 = robot.forward_kinematics(angles_3)
        print(f"Angles: {angles_3} -> End Effector Position: {pos_3}")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
