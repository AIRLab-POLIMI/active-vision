## Igus ReBeL MoveIt2 Configuration

MoveIt2 configuration files for controlling the robot graphically with Rviz2. It contains the definitions for everything that is needed to control the robot.

It contains definitions for:
- OMPL planner and MoveitSimpleController Manager connections
- Kinematic constraints
- Velocity and acceleration constraints
- ROS2 control with joint trajectory controller
- SRDF configurations and XACRO macros for controls and collisions
- Joint positions for standard positions

### Usage

#### Rviz

To start MoveIt2 controlling the Igus ReBeL on Rviz:

``` bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py load_base:=false mount:=none camera:=none end_effector:=none
```

#### Gazebo Ignition and Rviz

To start MoveIt2 controlling the Igus ReBeL on Rviz and transfer the control also on the Gazebo Ignition spawned arm:
``` bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py load_base:=false mount:=none camera:=none end_effector:=none load_gazebo:=true hardware_protocol:=ignition
```

### Comments

The MoveIt2 process colors do not works properly. Infact sometime it can happen that the collisions are not visualized correclty with the red color, or the goal position of the robot is not marked as yellow. Hovewer, the actual control works properly despite this.