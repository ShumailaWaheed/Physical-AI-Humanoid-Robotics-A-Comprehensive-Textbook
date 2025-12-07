# Voice to Action Pipeline

## Commanding Your Robot with the Power of Your Voice

Building on the fundamentals of VLA models, this chapter delves into a specific and highly intuitive application: the **Voice to Action Pipeline**. This pipeline enables a robot to understand spoken human commands, translate them into actionable instructions, and then execute those instructions in its physical environment. This capability significantly enhances human-robot interaction, making robots more accessible and natural to command.

### 1. Overview of the Voice to Action Pipeline

The Voice to Action Pipeline typically consists of several interconnected stages:

1.  **Speech-to-Text (STT)**: Converts spoken audio into written text.
2.  **Natural Language Understanding (NLU)**: Parses the text to extract the user's intent, relevant entities, and parameters. Often performed by an LLM.
3.  **Command Translation/Planning**: Translates the understood intent into a sequence of low-level robot commands (e.g., ROS 2 services, actions, or topic publications). This is where the LLM's reasoning capabilities are crucial.
4.  **Robot Action Execution**: A ROS 2 interface executes the generated robot commands.
5.  **Feedback (Optional)**: The robot provides verbal or visual feedback to the user regarding the execution status.

### 2. Component Breakdown

#### a. Speech-to-Text (STT)

The first step is to convert human speech into text.
*   **Technologies**:
    *   **Cloud-based APIs**: Google Speech-to-Text, AWS Transcribe, Azure Speech Service. These offer high accuracy but require internet connectivity.
    *   **On-device models**: Models like OpenAI's Whisper (can be run locally), Vosk, or Picovoice allow for offline processing, crucial for edge robotics.
*   **Integration**: Typically, an audio capture node (e.g., a ROS 2 node using `pyaudio` or a dedicated microphone driver) streams audio to the STT service/model, which then publishes the recognized text to a ROS 2 topic.

#### b. Natural Language Understanding (NLU) and Command Translation (LLM)

This is the brain of the pipeline, where the recognized text is converted into robot actions. An LLM acts as the core NLU and planner.

**Key Idea: LLM as a Tool User**

The LLM needs to know what the robot *can* do. This is achieved by providing the LLM with "tool descriptions" of the robot's ROS 2 functionalities.

**Tool Description Example (for LLM Prompt):**

```
You are a robot assistant. Here are the functions you can call to control me:

# Move the robot to a specified (x, y) coordinate
move_to_point(x: float, y: float)

# Grasp an object by its name
grasp_object(object_name: str)

# Release the currently held object
release_object()

# Example conversation:
# Human: "Go to the kitchen"
# Assistant: call: move_to_point(5.0, 2.0)

# Human: "Pick up the red ball"
# Assistant: call: grasp_object("red ball")
```

**Workflow:**
1.  The STT output (text) is fed into the LLM as part of a prompt.
2.  The LLM, based on the provided tool descriptions, generates a function call (e.g., `move_to_point(1.0, 0.5)`) or a sequence of calls.
3.  This LLM output (often in a JSON or specific function call format) is then published to a ROS 2 topic.

#### c. Robot Action Execution (ROS 2 Interface)

A dedicated ROS 2 node subscribes to the LLM's command topic. This node is responsible for parsing the LLM's output and executing the corresponding ROS 2 actions, services, or topic publications.

**Example: ROS 2 Command Executor Node (Python)**

```python
# command_executor_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # For LLM commands (e.g., JSON string)
from geometry_msgs.msg import Twist # For robot movement
# from my_ros2_package.srv import GraspObject # Assuming custom service
import json

class CommandExecutor(Node):
    def __init__(self):
        super().__init__('command_executor')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.grasp_client = self.create_client(GraspObject, 'grasp_object_service') # Example service client
        self.subscription = self.create_subscription(
            String,
            'llm_robot_commands',
            self.llm_command_callback,
            10)
        self.get_logger().info('Command Executor Node started.')

    def llm_command_callback(self, msg):
        try:
            command_data = json.loads(msg.data)
            function_name = command_data.get('function')
            args = command_data.get('args', {})

            if function_name == 'move_forward':
                twist_msg = Twist()
                twist_msg.linear.x = float(args.get('speed', 0.2))
                self.cmd_vel_publisher.publish(twist_msg)
                self.get_logger().info(f"Executing move_forward with speed: {twist_msg.linear.x}")
            elif function_name == 'turn_left':
                twist_msg = Twist()
                twist_msg.angular.z = float(args.get('angle_speed', 0.5))
                self.cmd_vel_publisher.publish(twist_msg)
                self.get_logger().info(f"Executing turn_left with angle_speed: {twist_msg.angular.z}")
            # Add more robot actions here, e.g., service calls
            # elif function_name == 'grasp_object':
            #     req = GraspObject.Request()
            #     req.object_name = args.get('object_name')
            #     self.grasp_client.call_async(req)
            #     self.get_logger().info(f"Calling grasp_object service for {req.object_name}")
            else:
                self.get_logger().warn(f"Unknown or unsupported command from LLM: {function_name}")

        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse LLM command JSON: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing LLM command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Considerations for Robustness

*   **Error Handling**: What happens if the LLM generates an invalid command?
*   **Ambiguity Resolution**: How does the robot ask for clarification if a command is ambiguous?
*   **Safety**: Ensuring the LLM does not generate unsafe commands.
*   **Context Management**: Maintaining a dialogue history with the LLM to understand multi-turn conversations.

### Next Steps

The Voice to Action Pipeline is a powerful demonstration of how language models can empower robots. However, language alone isn't always enough. The next chapter will explore how robots can use **Cognitive Planning with LLMs** to reason about tasks, break them down into sub-goals, and generate more complex, multi-step plans.
