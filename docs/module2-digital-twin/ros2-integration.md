# ROS 2 Integration with Simulators

## Connecting Your Robotic Brain to the Virtual Body

The true power of digital twins in robotics is unleashed when they can be seamlessly controlled and monitored by the same software stack used for physical robots. For ROS 2, this means establishing robust communication bridges between your ROS 2 nodes and both Gazebo and Unity simulation environments. This chapter details the essential tools and techniques for integrating ROS 2 with these simulators.

### 1. ROS 2 and Gazebo Integration

Gazebo's integration with ROS 2 is deep and well-established, primarily facilitated by the `gazebo_ros_pkgs` and `ros_gz_bridge`.

#### a. `gazebo_ros_pkgs`

These packages provide a set of plugins and utilities that allow ROS 2 nodes to interact with Gazebo. Key components include:

*   **`libgazebo_ros_init.so`**: Initializes the ROS 2 node within Gazebo itself, allowing direct ROS 2 access to Gazebo's internal APIs.
*   **`libgazebo_ros_factory.so`**: Enables spawning Gazebo models from ROS 2 using services.
*   **Sensor Plugins**: As seen in Chapter 11, plugins like `libgazebo_ros_camera.so`, `libgazebo_ros_imu_sensor.so`, `libgazebo_ros_ray_sensor.so` publish sensor data directly to ROS 2 topics.
*   **Actuator Plugins**: Plugins that allow ROS 2 to control actuators, such as joint controllers (e.g., `libgazebo_ros_joint_state_publisher.so` or `gazebo_ros_control` if using `ros2_control`).

**Example: Spawning a Robot from ROS 2**

In Chapter 10, we used `spawn_entity.py`. This script is part of `gazebo_ros`, and it communicates with Gazebo's `SpawnEntity` service. A common workflow involves a launch file that:
1.  Starts Gazebo with a specific world.
2.  Launches `robot_state_publisher` to publish the robot's URDF.
3.  Calls `spawn_entity.py` to insert the robot into the Gazebo world.

#### b. `ros_gz_bridge`

The `ros_gz_bridge` package provides a generic method to bridge messages between ROS 2 and Gazebo's native communication system (Gazebo Transport). This is particularly useful for connecting new or custom Gazebo topics/messages to ROS 2.

**Bridging Example: ROS 2 Twist to Gazebo cmd_vel**

If your Gazebo model uses a plugin that expects Gazebo Transport messages for velocity commands, you can bridge a ROS 2 `Twist` message to it.

First, identify the Gazebo Transport topic (e.g., `/model/my_robot/cmd_vel`) and its type (e.g., `gz.msgs.Twist`).
Then, in a ROS 2 launch file, you can declare a bridge:

```python
# Minimal example in a launch file
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='ros_gz_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist', # ROS 2 topic to Gazebo topic
                '/model/my_robot/pose@tf2_msgs/msg/TF[gz.msgs.Pose', # Gazebo pose to ROS 2 TF
            ],
            output='screen'
        )
    ])
```

The syntax `/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist` indicates:
*   ROS 2 topic: `/cmd_vel`
*   ROS 2 message type: `geometry_msgs/msg/Twist`
*   Gazebo message type: `gz.msgs.Twist`

### 2. ROS 2 and Unity Integration

Integrating ROS 2 with Unity is achieved through a set of official Unity Robotics packages, primarily `ROS-TCP-Endpoint` and `ROS-Unity-Integration`.

#### a. ROS-TCP-Endpoint

This package creates a TCP/IP endpoint within your Unity application that can communicate with a ROS 2 system. It acts as the low-level transport layer, allowing messages to flow between Unity and ROS 2.

**Setup**:
1.  In your ROS 2 environment, ensure `ros2_bridge` is running if you need to bridge ROS 1 to ROS 2, or if you're using `ROS-TCP-Endpoint` with ROS 1. For pure ROS 2, `ROS-TCP-Endpoint` communicates directly with Unity via TCP.
2.  In Unity, the `ROSConnection` component (from `ROS-TCP-Endpoint`) manages the TCP connection. You set the ROS 2 (or ROS Bridge) IP address and port.

#### b. ROS-Unity-Integration

This package provides C# classes that mirror your ROS 2 message types, allowing you to create publishers and subscribers within Unity that speak directly to ROS 2 topics.

**Workflow**:
1.  **Generate C# Messages**: The `ROS-Unity-Integration` tools can generate C# message definitions from your ROS 2 `.msg`, `.srv`, and `.action` files. This step makes your ROS 2 data types available in Unity's C# scripting environment.
2.  **Create Publishers/Subscribers in Unity**: Write C# scripts that use the generated message types and `ROSConnection` to publish data from Unity sensors (e.g., virtual camera, virtual LiDAR) to ROS 2 topics, or subscribe to ROS 2 topics (e.g., `/cmd_vel`) to control your Unity robot.

**Example: Unity Publisher for Odometry to ROS 2**

```csharp
// UnityOdomPublisher.cs
using RosMessageTypes.Nav; // Assumes nav_msgs/Odometry.msg is generated
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class UnityOdomPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string rosTopicName = "/odom"; // ROS 2 topic for odometry
    private NavOdometryMsg odomMessage;

    void Start()
    {
        ros = ROSConnection.Get  Instance();
        ros.RegisterPublisher<NavOdometryMsg>(rosTopicName);
        odomMessage = new NavOdometryMsg();
    }

    void FixedUpdate() // Or Update, depending on your needs
    {
        // Populate odomMessage with current robot pose and velocity from Unity
        // Example: Get pose from Unity's Transform component
        odomMessage.pose.pose.position.x = transform.position.x;
        odomMessage.pose.pose.position.y = transform.position.y;
        odomMessage.pose.pose.position.z = transform.position.z;
        // ... fill orientation, covariance, twist data ...

        ros.Publish(rosTopicName, odomMessage);
    }
}
```

### Next Steps

By successfully integrating ROS 2 with both Gazebo and Unity, you now have a powerful digital twin development pipeline. You can use ROS 2 to command your virtual robots, receive realistic sensor feedback, and develop complex AI algorithms in a safe and reproducible environment. The next module will take you deeper into the AI side, exploring how NVIDIA Isaac can be used as the "brain" for your robots.
