# Gazebo Simulation Setup

## Your Virtual Robot Testbed

Gazebo is one of the most powerful and widely used 3D robot simulators in the ROS 2 ecosystem. It allows you to accurately simulate complex robotic systems, their environments, and sensor feedback. This chapter will guide you through setting up Gazebo, understanding its basic interface, and loading a simple robot model into a simulated world.

### 1. Installing Gazebo (Garden Distribution)

For ROS 2 Humble (the recommended environment for this book), we typically use Gazebo Garden.

```bash
# Update package list
sudo apt update

# Install Gazebo Garden
sudo apt install ros-humble-gazebo-ros-pkgs # This installs Gazebo Garden and its ROS 2 integration packages
```

Verify your installation by running Gazebo:

```bash
gazebo # This should launch the Gazebo GUI
```

### 2. Understanding the Gazebo Interface

When Gazebo launches, you'll see a 3D simulation environment. Key components of the GUI include:

*   **World View**: The main area displaying your robot and environment.
*   **Scene Tree (Left Panel)**: Lists all entities (models, lights, ground plane) in the current world.
*   **Insert Panel (Left Panel)**: Allows you to add primitive shapes or pre-made models from online repositories (e.g., fuels.gazebosim.org).
*   **Toolbar (Top)**: Contains tools for interacting with the simulation (pause, play, step, move, rotate objects).
*   **Status Bar (Bottom)**: Displays simulation time, real-time factor, and other useful information.

Familiarize yourself with basic navigation:
*   **Pan**: Middle mouse button or Shift + Left mouse button.
*   **Rotate**: Left mouse button.
*   **Zoom**: Scroll wheel.

### 3. Creating a Simple World File

A Gazebo `.world` file defines the environment in which your robot will operate. It can include objects, terrain, lighting, and physics properties. Let's create a minimal world.

Create a file `simple_world.world` in your ROS 2 package (e.g., `my_ros2_package/worlds/simple_world.world`):

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_empty_world">
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <sun>
      <pose>0 0 10 0 0 0</pose>
    </sun>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

To launch this world:

```bash
gazebo -s libgazebo_ros_factory.so my_ros2_package/worlds/simple_world.world
```

The `-s libgazebo_ros_factory.so` option loads the Gazebo-ROS bridge plugin, which is essential for ROS 2 to interact with Gazebo.

### 4. Loading a URDF Robot Model into Gazebo

In the previous chapter, we created a simple `my_robot.urdf`. To load this into Gazebo, we typically use a ROS 2 launch file.

Create a launch file `robot_spawn.launch.py` in your package (e.g., `my_ros2_package/launch/robot_spawn.launch.py`):

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to your URDF file
    urdf_file_name = 'my_robot.urdf' # Assuming your URDF is in the root of your package share directory
    urdf_path = os.path.join(
        get_package_share_directory('my_ros2_package'),
        urdf_file_name
    )

    # Robot State Publisher node
    # Publishes the robot's state from the URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': LaunchConfiguration('robot_description')}],
        arguments=[urdf_path]
    )

    # Spawn the robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_description',
            default_value='<robot name="my_robot"></robot>', # This is a placeholder, actual content from URDF
            description='URDF description of the robot to spawn'
        ),
        robot_state_publisher_node,
        spawn_entity
    ])
```

**Note**: The `robot_description` argument in `robot_state_publisher` is often populated directly from the URDF content for complex models, but for simplicity here, `arguments=[urdf_path]` can work if the `robot_description` parameter is directly set. A more robust way is to read the URDF file content into `LaunchConfiguration('robot_description')`.

To launch the robot in Gazebo (you need to have Gazebo running with `gazebo my_ros2_package/worlds/simple_world.world` first, or integrate this into a single launch file):

```bash
ros2 launch my_ros2_package robot_spawn.launch.py
```

After running this, you should see your `my_robot` appear in the Gazebo simulation.

### Next Steps

Now that you can set up Gazebo and load your robot models, the next chapter will delve into simulating realistic sensors and physics, which are crucial for developing robust robot behaviors.
