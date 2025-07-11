# KUKA-robot-Trajectory-Planning
### KUKA LBR iiwa 7 R800 Robotic Arm Trajectory Planning and Execution

This repository includes MATLAB implementations for trajectory planning, forward kinematics, and velocity profiling of a 7-degree-of-freedom KUKA LBR iiwa 7 R800 robotic arm. The primary objective is to precisely maneuver the robotic arm to position a rectangular object accurately onto a designated rectangular target, guided by visual detection using a camera and an Aruco marker.

#### Project Overview:

* **Robot Specifications:** 7-DOF KUKA LBR iiwa robotic arm equipped with an adaptor featuring a rectangular object on one side and a camera on the other.
* **Objective:** Position the rectangular object onto a marked target within workspace constraints.
* **Vision System:** Utilizes a camera detecting an Aruco marker for accurate target localization and orientation determination.

#### Key Features and Implementation Details:

* **Forward Kinematics (FK):**

  * Calculates transformation matrices for each joint using defined joint offsets, rotation angles, and link lengths.
  * Computes comprehensive transformations (Tbt, Tef, Tbe) for precise end-effector positioning.

* **Trajectory Generation:**

  * Employs iterative methods to adjust joint angles, ensuring the end-effector reaches the desired pose.
  * Computes the Jacobian matrix to correlate joint velocities with end-effector velocities.

* **Coordinated Path Generation:**

  * Utilizes quintic polynomial interpolation for smooth transition between initial and final joint configurations.
  * Generates polynomial coefficients to ensure continuous and smooth joint trajectories stored in the PoseG matrix.

* **Velocity Profile Calculation:**

  * Derives velocity profiles by differentiating the joint trajectories.
  * Ensures velocities remain within predefined joint limits and motion constraints, recorded in the VeloG matrix.

* **Constraints and Safety:**

  * Ensures joint positions and velocities adhere strictly to specified safety and operational limits.
  * Carefully manages the robotic arm's movements to avoid collisions with surfaces and other obstacles within the workspace.

#### Results and Validation:

* Successfully generates smooth, safe, and accurate trajectories.
* Achieves precise end-effector placement guided by visual feedback.
* Validates final joint positions, velocities, and adherence to trajectory constraints through comprehensive analysis and visualization.

#### Future Enhancements:

* Integrating real-time obstacle avoidance and adaptive trajectory adjustment.
* Enhancing precision through advanced visual servoing techniques.
* Implementing robust control strategies for handling uncertainties in dynamic environments.


