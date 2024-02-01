## Igus ReBeL Description

This package contains the description of the Igus ReBeL 6 DOF robot arm.

### Usage

#### Rviz

To visualize the description of the Igus ReBeL only on Rviz:

```bash
$ ros2 launch igus_rebel_description_ros2 visualize.launch.py load_base:=false mount:=none end_effector:=none camera:=none
```


#### Gazebo Ignition and Rviz

To visualize the description of the Igus ReBeL also in Gazebo Ignition:

```bash
$ ros2 launch igus_rebel_description_ros2 visualize.launch.py load_base:=false mount:=none end_effector:=none camera:=none load_gazebo:=true
```

When the Igus ReBeL is loaded in Gazebo Ignition, it is possible to control and move it by using the joint position controller gui, located on the top left side of Gazebo Ignition. By clicking the Igus ReBeL inside the simulation, the joints will appear inside the joint position controller gui, and it is possible to change their values. The movement in Gazebo Ignition will be also transferred to Rviz.