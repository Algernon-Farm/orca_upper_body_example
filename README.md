# orca_upper_body_example

## Install

- [LCM](https://lcm-proj.github.io/lcm/content/install-instructions.html)

## Structure

```text
├── controller2robot.py                 # example of sending commands to the robot from a controller
├── LICENSE
├── lowlevel_sdk                        # definitions of LCM types for the robot
│   ├── cyan_armwaisthead_cmd_lcmt.py
│   ├── cyan_armwaisthead_data_lcmt.py
│   └── __init__.py
├── README.md
└── robot2controller.py                 # example of sending data from the robot to a controller
```