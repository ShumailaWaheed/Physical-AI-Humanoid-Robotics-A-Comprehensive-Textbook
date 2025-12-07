# Research & Decisions

This document records the key decisions made during the planning phase for the "Physical AI & Humanoid Robotics Book" feature.

## Module Sequencing

**Decision**: The book's modules will be sequenced as follows:
1.  **Module 1: The Robotic Nervous System (ROS 2)**
2.  **Module 2: The Digital Twin (Gazebo & Unity)**
3.  **Module 3: The AI-Robot Brain (NVIDIA Isaac)**
4.  **Module 4: Vision-Language-Action (VLA) + Capstone**

**Rationale**: This sequence follows a logical progression from foundational concepts to advanced applications. Readers must first understand the "nervous system" (ROS 2) before they can give it a "body" in simulation (Gazebo/Unity). The "brain" (Isaac) builds on the simulated body, and high-level cognitive tasks (VLA) require the brain and body to be in place. This structure ensures prerequisites are met at each stage.

**Alternatives considered**: A parallel structure was considered but rejected because the technologies have clear dependencies on each other, making a sequential learning path more effective for students.

## Hardware Assumptions

**Decision**: The book will assume the reader has a modern PC capable of running GPU-intensive simulations and will provide guidance for using an NVIDIA Jetson for edge-based examples where applicable.

- **Primary**: A desktop or laptop with a dedicated NVIDIA GPU (e.g., RTX 3060 or higher), 16GB+ RAM, and running Ubuntu 22.04.
- **Secondary (for specific chapters)**: An NVIDIA Jetson (e.g., AGX Orin or Orin Nano) for demonstrating deployment on edge hardware.

**Rationale**: This provides a balance. The powerful PC is necessary for a smooth experience with tools like Isaac Sim and Gazebo. The Jetson is a popular and relevant platform for robotics education and research, and including it adds significant practical value.

**Alternatives considered**: Assuming only a low-powered laptop was rejected as it would prevent users from completing the core simulation exercises. Assuming only a real robot was rejected as it would create a high barrier to entry.

## Simulation vs. Real-Robot Examples

**Decision**: The book will primarily focus on simulation-based examples. Real-robot examples will be discussed conceptually or provided as supplementary material/extensions for readers with access to hardware.

**Rationale**: Simulation provides a low-cost, accessible, and repeatable way for all readers to complete the exercises. It removes the significant barrier of hardware procurement and setup. This aligns with the goal of creating a widely accessible educational resource.

**Alternatives considered**: A real-robot-first approach was rejected due to the high cost and complexity, which would limit the book's audience.

## Pseudocode vs. Full Code Templates

**Decision**: The book will use a hybrid approach:
- **Core Concepts**: Full, working code templates and snippets will be provided for fundamental concepts (e.g., creating a ROS 2 node, publishing a topic).
- **Complex Systems**: Pseudocode will be used to describe the high-level logic of more complex systems (e.g., the entire cognitive planning loop), with full code for key interfaces.

**Rationale**: This approach ensures that students can learn the fundamentals with concrete, runnable examples. For highly complex or project-specific code, pseudocode is more effective for teaching the *logic and architecture* without overwhelming the reader with boilerplate or implementation details that might vary.

**Alternatives considered**: Providing full source code for everything was rejected as it would turn the book into a code dump and detract from the conceptual lessons. Using only pseudocode was rejected as it would not give students enough concrete, hands-on experience.

## Choice of Diagrams

**Decision**: The book will utilize a mix of diagram types to explain different concepts:
- **Workflow Diagrams**: To show the flow of data or execution in a process (e.g., the VLA pipeline).
- **Architectural Diagrams**: To illustrate the high-level components of a system and their relationships (e.g., how ROS 2, Gazebo, and an LLM connect).
- **Sensor Integration Diagrams**: To show how simulated sensors connect to the robot model and publish data.

**Rationale**: Different concepts require different types of visualization. Using a variety of clear, well-annotated diagrams will significantly improve comprehension for visual learners. All diagrams will follow a consistent style.

**Alternatives considered**: Using only one type of diagram was rejected as it would be insufficient to clearly explain the variety of technical concepts.
