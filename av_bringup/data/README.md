# Ground Truth Details

There are 4 ground truth corresponding to 4 different tomato plants:
- `octree_tomato_1.bt` contains the ground truth of the reconstructed octree related to the occluded tomato plant, placed at coordinates (0.0, 0.0). This ground truth has been saved with the robot arm placed at coordinates (-0.6, 0.0).
- `octree_tomato_2.bt` contains the ground truth of the reconstructed octree related to the not occluded tomato plant, placed at coordinates (0.0, 3.0). This ground truth has been saved with the robot arm placed at coordinates (-0.7, 3.0).
- `octree_tomato_3.bt` contains the ground truth of the reconstructed octree related to the tomato plant containing few fruits, placed at coordinates (0.0, 6.0). This ground truth has been saved with the robot arm placed at coordinates (-0.7, 6.0).
- `octree_tomato_4.bt` contains the ground truth of the reconstructed octree related to the not occluded tomato plant, placed at coordinates (0.0, 3.0). This ground truth has been saved with the robot arm placed at unoriented with respect to the plant, at coordinates (-0.7, 2.6).

To test the active vision pipeline using these ground truth, it is necessary to spawn the robotic arm in Gazebo at the same coordinate where the ground truth has been recorded (using command line parameters `spawn_x`, `spawn_y`)

## Save new ground truth

- Firstly, it is needed to execute the octomap creation node to simulate the environment in Gazebo: `ros2 launch av_bringup octomap_normal.launch.py run_robot:=true run_rviz:=true run_pt:=true run_octomap:=true load_base:=false mount:=mount_v2 camera:=realsense end_effector:=none`
  - Change spawn position to move the robot, getting a different ground truth
- Move the robot to get the complete octomap in front of it
- Call the service to save the octomap: `ros2 service call /save_octomap_data_service std_srvs/srv/SetBool "{data: true}"`
  - The data is saved into the `install` folder of the workspace, inside the `av_bringup/data` package folder 