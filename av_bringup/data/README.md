## Ground Truth Details

There are 4 ground truth corresponding to 4 different tomato plants:
- `octree_tomato_1.bt` contains the ground truth of the reconstructed octree related to the occluded tomato plant, placed at coordinates (0.0, 0.0). This ground truth has been saved with the robot arm placed at coordinates (-0.6, 0.0).
- `octree_tomato_2.bt` contains the ground truth of the reconstructed octree related to the not occluded tomato plant, placed at coordinates (0.0, 3.0). This ground truth has been saved with the robot arm placed at coordinates (-0.7, 3.0).
- `octree_tomato_3.bt` contains the ground truth of the reconstructed octree related to the tomato plant containing few fruits, placed at coordinates (0.0, 6.0). This ground truth has been saved with the robot arm placed at coordinates (-0.7, 6.0).
- `octree_tomato_4.bt` contains the ground truth of the reconstructed octree related to the not occluded tomato plant, placed at coordinates (0.0, 3.0). This ground truth has been saved with the robot arm placed at unoriented with respect to the plant, at coordinates (-0.7, 2.6).

To test the active vision pipeline using these ground truth, it is necessary to spawn the robotic arm in Gazebo at the same coordinate where the ground truth has been recorded (using command line parameters `spawn_x`, `spawn_y`)