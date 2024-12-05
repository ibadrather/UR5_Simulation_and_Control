# Joint Control Nodes for a Robotic Arm

This repository contains two ROS nodes: `sin_mover` and `cart_pos`. Both nodes interact with a 6-joint robotic arm, but they serve distinct purposes and operate differently.

## Node Descriptions

### 1. **`sin_mover`**
The `sin_mover` node continuously publishes sinusoidal position commands to all six joints of the robotic arm. This generates smooth, oscillatory motions in the joints, which can be used for testing joint responsiveness or showcasing the arm's movement capabilities.

#### How It Works:
- The node initializes publishers for each joint's position command topic.
- It calculates joint angles using a sinusoidal function of time.
- These calculated angles are published at a fixed frequency (100 Hz) to move the joints smoothly.

#### Key Topics:
- **Publishers:**
  - `/shoulder_pan_joint_position_controller/command`
  - `/shoulder_lift_joint_position_controller/command`
  - `/elbow_joint_position_controller/command`
  - `/wrist_1_joint_position_controller/command`
  - `/wrist_2_joint_position_controller/command`
  - `/wrist_3_joint_position_controller/command`

---

### 2. **`cart_pos`**
The `cart_pos` node enables Cartesian position-based control of the robotic arm. It reads the current joint positions, computes desired Cartesian target positions, and updates the joint positions accordingly.

#### How It Works:
- Subscribes to the joint state topics to read the current positions of all joints.
- Uses the KDL library to parse the URDF model and generate a kinematic chain for the robotic arm.
- Accepts user inputs for Cartesian offsets and computes the target position for the robot's end-effector.
- Solves inverse kinematics (IK) to calculate the required joint angles to achieve the desired Cartesian position.
- Publishes the computed joint commands to move the arm.

#### Key Topics:
- **Subscribers:**
  - `/shoulder_pan_joint_position_controller/state`
  - `/shoulder_lift_joint_position_controller/state`
  - `/elbow_joint_position_controller/state`
  - `/wrist_1_joint_position_controller/state`
  - `/wrist_2_joint_position_controller/state`
  - `/wrist_3_joint_position_controller/state`
- **Publishers:**
  - `/shoulder_pan_joint_position_controller/command`
  - `/shoulder_lift_joint_position_controller/command`
  - `/elbow_joint_position_controller/command`
  - `/wrist_1_joint_position_controller/command`
  - `/wrist_2_joint_position_controller/command`
  - `/wrist_3_joint_position_controller/command`

---
