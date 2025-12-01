# Forward Kinematics for 4-DOF Robot Arm

This repository contains a Python implementation of the forward kinematics for a specific 4-DOF robot arm.

## Robot Description

- **Joints:** 4 Revolute joints (J1, J2, J3, J4).
- **Links:** 4 Links of length `L` (default 1m).
- **Configuration:** The axis of each joint is perpendicular to the previous joint.

## Prerequisites

- Python 3
- NumPy

## Installation

1. Clone the repository.
2. Install NumPy if not already installed:
   ```bash
   pip install numpy
   ```

## Usage

You can run the script directly to see example outputs:

```bash
python3 forward_kinematics.py
```

### Using the Class

```python
from forward_kinematics import RobotArm
import numpy as np

# Initialize the robot with link length L=1.0 meters
robot = RobotArm(link_length=1.0)

# Define joint angles in radians [j1, j2, j3, j4]
joint_angles = [0, np.pi/4, -np.pi/4, 0]

# Calculate end-effector position
position = robot.forward_kinematics(joint_angles)

print(f"End Effector Position (x, y, z): {position}")
```

## Mathematical Explanation

The solution uses the **Denavit-Hartenberg (DH)** convention to model the robot.

### DH Parameters

Based on the requirement that each joint axis is perpendicular to the previous one, and links have length $L$, we assigned the following DH parameters:

| Link ($i$) | $\theta_i$ (Rotation about $Z_{i-1}$) | $d_i$ (Translation along $Z_{i-1}$) | $a_i$ (Translation along $X_i$) | $\alpha_i$ (Rotation about $X_i$) |
| :--- | :--- | :--- | :--- | :--- |
| 1 | $j_1$ | 0 | $L$ | $90^\circ$ |
| 2 | $j_2$ | 0 | $L$ | $90^\circ$ |
| 3 | $j_3$ | 0 | $L$ | $90^\circ$ |
| 4 | $j_4$ | 0 | $L$ | $0^\circ$ |

*Note: The choice of $\alpha = 90^\circ$ for all links satisfies the perpendicularity constraint. Other configurations (e.g., alternating $\pm 90^\circ$) are also possible interpretations of the diagram but yield similar kinematic structures.*

### Transformation Matrix

The transformation matrix from frame $i-1$ to $i$ is given by:

$$
T_{i-1}^{i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

The final position of the end-effector is obtained by multiplying the transformation matrices:

$$ T_{base}^{end} = T_0^1 \cdot T_1^2 \cdot T_2^3 \cdot T_3^4 $$

The position vector is the last column of $T_{base}^{end}$.
