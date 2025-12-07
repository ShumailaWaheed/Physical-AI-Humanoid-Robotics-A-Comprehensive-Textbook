# Building ROS 2 Packages (Python)

## Structuring Your Robot Software

In ROS 2, a **package** is the fundamental unit for organizing your code. It's a directory containing source code, data, dependencies, and configuration files that together provide specific functionality. This chapter will guide you through creating a new ROS 2 package in Python, defining custom message types, and configuring it for building and installation.

### 1. Creating a New ROS 2 Package

We'll use `ros2 pkg create` to generate the basic structure for a Python package.

```bash
# Navigate to your ROS 2 workspace src directory (e.g., ~/ros2_ws/src)
cd ~/ros2_ws/src

# Create a new Python package named 'my_ros2_package'
ros2 pkg create --build-type ament_python my_ros2_package
```

This command creates a directory `my_ros2_package` with the following basic structure:

```
my_ros2_package/
├── my_ros2_package/
│   └── __init__.py
├── package.xml
└── setup.py
```

*   `my_ros2_package/`: This inner directory will hold your Python modules.
*   `__init__.py`: Marks the directory as a Python package.
*   `package.xml`: Defines package metadata, dependencies, and build information.
*   `setup.py`: The Python build script for installing your package's Python modules and entry points.

### 2. Defining Custom Messages, Services, and Actions

While standard ROS 2 messages (`std_msgs`) are useful, you'll often need to define custom data types for your application. These are defined in `.msg`, `.srv`, and `.action` files.

#### Custom Message Example (`my_ros2_package/msg/CustomPosition.msg`)

Let's say we want a message to represent a 3D position with an associated ID.

```
# CustomPosition.msg
int32 id
float64 x
float64 y
float64 z
```

#### Custom Service Example (`my_ros2_package/srv/SetLed.srv`)

A service to set the state of an LED and get a success response.

```
# SetLed.srv
int32 led_number
bool state
---
bool success
string message
```

#### Custom Action Example (`my_ros2_package/action/MoveRobot.action`)

An action to move a robot to a target position, providing feedback along the way.

```
# MoveRobot.action
float32 target_x
float32 target_y
float32 target_z
---
bool success
float32 final_x
float32 final_y
float32 final_z
---
float32 current_x
float32 current_y
float32 current_z
```

Place these files in `my_ros2_package/msg`, `my_ros2_package/srv`, and `my_ros2_package/action` directories (you'll need to create these directories).

### 3. Updating `package.xml`

You need to tell ROS 2 that your package defines custom messages, services, and actions, and that it depends on `rosidl_default_generators` to generate the necessary code.

Add the following lines inside the `<build_depend>` and `<exec_depend>` sections of your `package.xml`:

```xml
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

Also, ensure you declare a dependency on `std_msgs` if you plan to use standard messages.

### 4. Updating `setup.py`

The `setup.py` script is crucial for building and installing your Python package. For custom message types, you need to configure `setup.py` to tell ROS 2's build system (`ament`) about them.

Open `setup.py` and modify it as follows:

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_ros2_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.[pxy][yeml]'))), # Example for launch files
        # Add these lines for custom messages, services, actions
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
        (os.path.join('share', package_name, 'action'), glob('action/*.action')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add entry points for your nodes here, e.g.:
            # 'my_node = my_ros2_package.my_node:main',
        ],
    },
)
```

**Key additions to `setup.py`:**

*   `data_files`: This section ensures that your `.msg`, `.srv`, and `.action` files are installed correctly so `rosidl_default_generators` can find them and generate the corresponding Python code.
*   `entry_points`: This is where you declare your executable Python scripts (nodes) so they can be run using `ros2 run`.

### 5. Building and Installing Your Package

Once `package.xml` and `setup.py` are configured, navigate back to your workspace root (`~/ros2_ws/`) and build your package:

```bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_package
```

After a successful build, you need to source your workspace to make the new package available in your environment:

```bash
source install/setup.bash # or setup.zsh for Zsh users
```

Now, you can check if your custom interfaces were generated:

```bash
ros2 interface show my_ros2_package/msg/CustomPosition
ros2 interface show my_ros2_package/srv/SetLed
ros2 interface show my_ros2_package/action/MoveRobot
```

You can also run any nodes you defined in `entry_points` of `setup.py`.

### Next Steps

With your package created and custom interfaces defined, you're ready to start implementing complex robot behaviors using these new data types. The next chapter will cover how to describe your robot's physical structure using URDF.
