import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * The sidebars object allows you to group documents into logical categories,
 * making it easier for users to navigate your documentation.
 */
const sidebars: SidebarsConfig = {
  // Define the sidebar for the book, named 'bookSidebar'.
  bookSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: [
        {type: 'doc', id: 'introduction/what-is-physical-ai', label: 'Chapter 1: What is Physical AI?'},
        {type: 'doc', id: 'introduction/why-humanoid-robotics-matters', label: 'Chapter 2: Why Humanoid Robotics Matters'},
        {type: 'doc', id: 'introduction/course-overview', label: 'Chapter 3: Course Overview & Learning Goals'},
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        {type: 'doc', id: 'module1-ros2/ros2-fundamentals', label: 'Chapter 4: ROS 2 Fundamentals'},
        {type: 'doc', id: 'module1-ros2/nodes-topics-services-actions', label: 'Chapter 5: Nodes, Topics, Services, Actions'},
        {type: 'doc', id: 'module1-ros2/building-ros2-packages', label: 'Chapter 6: Building ROS 2 Packages (Python)'},
        {type: 'doc', id: 'module1-ros2/urdf-and-robot-description', label: 'Chapter 7: URDF and Robot Description'},
        {type: 'doc', id: 'module1-ros2/connecting-llm-agents', label: 'Chapter 8: Connecting LLM Agents to ROS 2'},
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        {type: 'doc', id: 'module2-digital-twin/digital-twins-in-robotics', label: 'Chapter 9: Digital Twins in Robotics'},
        {type: 'doc', id: 'module2-digital-twin/gazebo-simulation-setup', label: 'Chapter 10: Gazebo Simulation Setup'},
        {type: 'doc', id: 'module2-digital-twin/sensors-and-physics', label: 'Chapter 11: Sensors and Physics Simulation'},
        {type: 'doc', id: 'module2-digital-twin/unity-for-robotics', label: 'Chapter 12: Unity for Robotics Visualization'},
        {type: 'doc', id: 'module2-digital-twin/ros2-integration', label: 'Chapter 13: ROS 2 Integration with Simulators'},
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        {type: 'doc', id: 'module3-ai-robot-brain/isaac-sim-overview', label: 'Chapter 14: Isaac Sim Overview'},
        {type: 'doc', id: 'module3-ai-robot-brain/synthetic-data-perception', label: 'Chapter 15: Synthetic Data & Perception Pipelines'},
        {type: 'doc', id: 'module3-ai-robot-brain/visual-slam', label: 'Chapter 16: Visual SLAM and Mapping'},
        {type: 'doc', id: 'module3-ai-robot-brain/navigation-and-planning', label: 'Chapter 17: Navigation and Planning with Isaac'},
        {type: 'doc', id: 'module3-ai-robot-brain/ai-driven-decision-making', label: 'Chapter 18: AI-Driven Robot Decision Making'},
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) + Capstone',
      items: [
        {type: 'doc', id: 'module4-vla-capstone/vla-fundamentals', label: 'Chapter 19: VLA Fundamentals'},
        {type: 'doc', id: 'module4-vla-capstone/voice-to-action', label: 'Chapter 20: Voice to Action Pipeline'},
        {type: 'doc', id: 'module4-vla-capstone/cognitive-planning', label: 'Chapter 21: Cognitive Planning with LLMs'},
        {type: 'doc', id: 'module4-vla-capstone/multi-modal-interaction', label: 'Chapter 22: Multi-Modal Interaction'},
        {type: 'doc', id: 'module4-vla-capstone/capstone-project-overview', label: 'Chapter 23: Capstone Project Overview'},
      ],
    },
  ],
};

export default sidebars;
