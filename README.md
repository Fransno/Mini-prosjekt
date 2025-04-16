# Qube Mini-Project

This project is a ROS2-based implementation for controlling a Quanser Qube via simulation and with hardware. The system combines URDF modeling, ROS2 Control integration, and a custom PID controller for real-time control and visualization in RViz.

## Packages

### `qube_description`
Contains the geometric and visual description of the Qube using URDF/Xacro.
- **`qube.macro.xacro`**: Macro definition of the Qube model
- **`qube.urdf.xacro`**: Simple scene placing the Qube in the world
- **Launch**: `view_qube.launch.py` visualizes the Qube in RViz with slider controls

### `qube_driver`
ROS2 Control interface to the physical Qube hardware.
- Uses USB/serial connection to a Teensy board
- Downloaded from external repo: [`qube_driver`](https://github.com/adamleon/qube_driver)

### `qube_bringup`
Launches the full system: model, driver, controllers, and visualization.
- **`controlled_qube.urdf.xacro`**: URDF including ROS2 control macros
- **`bringup.launch.py`**: Starts robot_state_publisher, controller manager, joint_state_broadcaster, velocity controller, and RViz. Supports dynamic hardware parameter configuration via launch arguments.
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
All critical hardware parameters can be changed without code modifications when launching (example):
```
ros2 launch qube_bringup bringup.launch.py \
    device:=/dev/ttyUSB0 \    # Change serial device (default: /dev/ttyACM0)
    baud_rate:=57600 \        # Change baud rate (default: 115200)
    simulation:=false         # Toggle hardware/simulation mode (default: true)
```
#### Troubleshooting 

If you encounter issues:

- Verify all packages are built (`colcon list`)   
- Check node status with : `ros2 node list`   
- Examine topic connections with : `ros2 topic list -t`   
