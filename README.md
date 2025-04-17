# Qube Mini-Project

This project is a ROS2-based implementation for controlling a Quanser Qube via simulation and with hardware. The system combines URDF modeling, ROS2 Control integration, and a custom PID controller for real-time control and visualization in RViz.

## Packages

### `qube_description`
Contains the geometric and visual description of the Qube using URDF/Xacro.
- **`qube.macro.xacro`**: Macro definition of the Qube model. Contains reusable parts of the model such as the base cube, spinning disk, and indicator. It serves as a template to create a Qube instance with different properties.
- **`qube.urdf.xacro`**: Simple scene placing the Qube in the world. Defines the Qube robot with some default properties (e.g., size, color) and is used for visualization and simulation.
- **Launch**: `view_qube.launch.py` visualizes the Qube in RViz with slider controls.

### `qube_driver`
ROS2 Control interface to the physical Qube hardware.
- Provides ROS2-compatible hardware interface for controlling the Qube hardware. This includes controlling the motor and reading the state of the robot (e.g., position, velocity). Uses USB/serial connection to a Teensy board
- Downloaded from external repo: [`qube_driver`](https://github.com/adamleon/qube_driver)

### `qube_bringup`
Launches the full system: model, driver, controllers, and visualization.
- **`controlled_qube.urdf.xacro`**: A modified version of the Qube URDF, incorporating ROS2 control macros to interface with hardware or simulation. 
- **`bringup.launch.py`**: Starts robot_state_publisher, joint_state_broadcaster, controllers and RViz. Supports dynamic hardware parameter configuration via launch arguments.
- **`controllers.yaml`**: Configuration file for controller types and PID gains

### `qube_controller`
Custom ROS2 node that implements a PID controller.
- **`pid_controller.cpp`**: The PID algorithm itself
- **`pid_controller_node.cpp`**: ROS2 node that subscribes to `/joint_states` and publishes to `/velocity_controller/commands`

## Usage

### Step 1: Building the Workspace

Navigate to your workspace root : 
```cd BRANCH-NAME```

Source the setup file :
```source install/setup.bash```

Build the packages (only needed after code changes) :
```colcon build```

### Step 2: Running the System

Launch the complete system (model, controllers, and visualization) :
```ros2 launch qube_bringup bringup.launch.py```

In a separate terminal, start the PID controller :
```ros2 run qube_controller pid_controller_node```

### Step 3: Monitering the Topics

View joint states (position, velocity, effort) :
```ros2 topic echo /joint_states```

View controller commands being sent :
```ros2 topic echo /velocity_controller/commands```

### Sending Manual Commands

Send a velocity command to the controller :
```
ros2 topic pub --once /velocity_controller/commands std_msgs/msg/Float64MultiArray "
layout:
  dim:
  - label: velocity
    size: 1
    stride: 1
  data_offset: 0
data: [0.5]  # Adjust this value as needed
"
```

### Configuration
You can change critical hardware parameters without modifying the code when launching the system.

1. Change hardware parameters at launch:
```
ros2 launch qube_bringup bringup.launch.py \
    device:=/dev/ttyUSB0 \    # Change serial device (default: /dev/ttyACM0)
    baud_rate:=57600 \        # Change baud rate (default: 115200)
    simulation:=false         # Toggle hardware/simulation mode (default: true)
```

2. Change PID parameters during runtime using ros2 param:
```
ros2 param set /pid_controller_node p 0.8  
ros2 param set /pid_controller_node i 0.05  
ros2 param set /pid_controller_node d 0.01  
ros2 param set /pid_controller_node reference 1.0  
```
#### Troubleshooting 

If you encounter issues:

- Verify all packages are built (`colcon list`)   
- Check node status with : `ros2 node list`   
- Examine topic connections with : `ros2 topic list -t`   
