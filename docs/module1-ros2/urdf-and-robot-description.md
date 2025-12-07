# URDF and Robot Description

## Defining Your Robot's Physical Form

To interact with the physical world, a robot needs a precise digital representation of its physical structure. In ROS 2, the **Unified Robot Description Format (URDF)** is the standard XML-based file format used to describe a robot's kinematic and dynamic properties. This chapter will introduce you to URDF, explain its core elements, and guide you through creating a simple URDF file for a mobile robot.

### What is URDF?

URDF defines a robot as a tree-like structure of **links** (rigid bodies) connected by **joints** (revolute or prismatic connections that allow motion). It can also describe the robot's visual appearance, collision properties, and inertial characteristics.

Key aspects of URDF:

*   **XML Format**: URDF files are structured using XML tags.
*   **Kinematic and Dynamic Properties**: Describes the robot's geometry, mass, inertia, and how its parts move relative to each other.
*   **Hierarchical Structure**: Robots are typically modeled as a tree, starting from a `base_link` and branching out.
*   **Visualization**: URDF is extensively used by visualization tools like RViz to display the robot model.
*   **Simulation**: Simulators like Gazebo can import URDF (often extended with Gazebo-specific tags) to simulate the robot's physics.

### Core URDF Elements

#### 1. `<robot>`

The root element of every URDF file, which encapsulates the entire robot description.

```xml
<robot name="my_simple_robot">
  <!-- Links and Joints go here -->
</robot>
```

#### 2. `<link>`

A `<link>` element describes a rigid body part of the robot.

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.2 0.2 0.05" />
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1" />
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.2 0.05" />
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0" />
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
  </inertial>
</link>
```

*   **`<visual>`**: Defines the graphical representation of the link (e.g., shape, color).
*   **`<collision>`**: Defines the collision geometry of the link. This can be simpler than the visual geometry for performance.
*   **`<inertial>`**: Defines the mass, center of mass, and inertia matrix, crucial for physics simulation.

#### 3. `<joint>`

A `<joint>` element describes how two links are connected and their relative motion.

```xml
<joint name="base_to_wheel_left" type="continuous">
  <parent link="base_link" />
  <child link="left_wheel_link" />
  <origin xyz="0.1 0.1 0.0" rpy="0 0 0" />
  <axis xyz="0 1 0" />
</joint>
```

*   **`name`**: Unique identifier for the joint.
*   **`type`**: Specifies the joint's movement (e.g., `revolute`, `continuous`, `prismatic`, `fixed`).
*   **`parent`**: The link closer to the robot's base.
*   **`child`**: The link further from the robot's base.
*   **`origin`**: Defines the joint's position and orientation relative to the parent link.
*   **`axis`**: For revolute/prismatic joints, specifies the axis of rotation/translation.

#### 4. `<gazebo>` Tags (for simulation)

While core URDF describes kinematics, for physics simulation in Gazebo, you often need to add Gazebo-specific extensions. These are usually embedded within a `<robot>` tag but not directly part of standard URDF.

```xml
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
```

### Example: A Simple Differential Drive Robot

Let's put it all together to define a basic mobile robot with a base and two wheels.

```xml
<!-- my_robot.urdf -->
<robot name="my_diff_drive_robot">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1" />
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1" />
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="5.0" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
    </inertial>
  </link>

  <!-- Left Wheel Link -->
  <link name="left_wheel_link">
    <visual>
      <origin xyz="0 0 0" rpy="1.5707 0 0" /> <!-- Rotate for cylinder orientation -->
      <geometry>
        <cylinder radius="0.05" length="0.02" />
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5707 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.02" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="1.5707 0 0" />
      <mass value="0.1" />
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001" />
    </inertial>
  </link>

  <!-- Right Wheel Link -->
  <link name="right_wheel_link">
    <visual>
      <origin xyz="0 0 0" rpy="1.5707 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.02" />
      </geometry>
      <material name="black" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5707 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.02" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="1.5707 0 0" />
      <mass value="0.1" />
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001" />
    </inertial>
  </link>

  <!-- Left Wheel Joint -->
  <joint name="base_to_left_wheel" type="continuous">
    <parent link="base_link" />
    <child link="left_wheel_link" />
    <origin xyz="0.1 0.1 0" rpy="0 0 0" />
    <axis xyz="0 1 0" />
  </joint>

  <!-- Right Wheel Joint -->
  <joint name="base_to_right_wheel" type="continuous">
    <parent link="base_link" />
    <child link="right_wheel_link" />
    <origin xyz="0.1 -0.1 0" rpy="0 0 0" />
    <axis xyz="0 1 0" />
  </joint>

  <!-- Materials for easier reuse -->
  <material name="blue">
    <color rgba="0 0 1 1" />
  </material>
  <material name="red">
    <color rgba="1 0 0 1" />
  </material>
  <material name="black">
    <color rgba="0 0 0 1" />
  </material>

</robot>
```

### Visualizing URDF

You can visualize URDF files using RViz, ROS's 3D visualization tool. First, you need to install `joint_state_publisher_gui`:

```bash
sudo apt install ros-humble-joint-state-publisher-gui
```

Then, you can launch it with your URDF file:

```bash
ros2 launch urdf_tutorial display.launch.py model:=path/to/my_robot.urdf
```

Replace `path/to/my_robot.urdf` with the actual path to your URDF file. This will open RViz, allowing you to manipulate the robot's joints and see its kinematic structure.

### Next Steps

Understanding URDF is fundamental for working with robot models in simulation and for controlling their physical counterparts. The next chapter will explore how Large Language Models (LLMs) can be integrated with ROS 2 to provide high-level command interfaces for these described robots.
