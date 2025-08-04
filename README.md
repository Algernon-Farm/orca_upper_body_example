# orca_upper_body_example

## Description

![](./assets/description.png)

## Install

- [LCM](https://lcm-proj.github.io/lcm/content/install-instructions.html)

## Structure

```text
.
├── cy_arm_controller_example
│   ├── assets
│   │   └── picture01.png
│   ├── cy_arm_controller_example
│   │   ├── __init__.py
│   │   ├── left_example.py
│   │   └── right_example.py
│   ├── package.xml
│   ├── README.md
├── cy_robot_dynamics_example
│   ├── assets
│   │   └── picture01.png
│   ├── cy_robot_dynamics_example
│   │   ├── __init__.py
│   │   ├── left_example.py
│   │   └── right_example.py
│   ├── package.xml
│   ├── README.md
├── LICENSE     
├── motor_control                           # motor control example 
│   ├── controller2robot.py                 # example of sending commands to the robot from a controller
│   ├── lowlevel_sdk                        # definitions of LCM types for the robot
│   │   ├── cyan_armwaisthead_cmd_lcmt.py   
│   │   ├── cyan_armwaisthead_data_lcmt.py
│   │   └── __init__.py
│   └── robot2controller.py                 # example of sending data from the robot to a controller
└── README.md
```