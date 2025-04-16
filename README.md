> [!IMPORTANT]
> This README is unfinished and below is just a unfinished semi-edited template originating from https://www.makeareadme.com/

# Foobar

Foobar is a Python library for dealing with word pluralization.

## Installation

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install foobar.

```bash
pip install foobar
```

## Usage

```console
cd BRANCH-NAME
source install/setup.bash
colcon build  # Only use after change in code (build command)

ros2 launch qube_bringup bringup.launch.py  # Start program

ros2 run qube_controller pid_controller_node   # Start PID controller


ros2 topic echo /joint_states  # Optional, see publishing information

ros2 topic echo /velocity_controller/commands  # Optional, see subscribing information


# Publishing command (changes position of spinning disk)
ros2 topic pub --once /velocity_controller/commands std_msgs/msg/Float64MultiArray "
layout:
  dim:
  - label: velocity
    size: 1
    stride: 1
  data_offset: 0
data: [0.5]  # Your desired value here
"

```

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License

[MIT](https://choosealicense.com/licenses/mit/)
