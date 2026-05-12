robot_pose_tf_bridge.py-11] [INFO] [1778058276.815252281] [robot_pose_tf_bridge]: Publishing TF pose map -> base_link to /robot_pose as Pose2D
[component_container_isolated-10] [INFO] [1778058276.827195901] [amcl]: Subscribed to map topic.
[spawner-5] [INFO] [1778058276.832727426] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[ros2_control_node-3] [INFO] [1778058276.833519170] [controller_manager]: Loading controller 'diff_drive_controller'
[component_container_isolated-10] [INFO] [1778058276.850541039] [lifecycle_manager_localization]: Activating map_server
[component_container_isolated-10] [INFO] [1778058276.850845783] [map_server]: Activating
[component_container_isolated-10] [INFO] [1778058276.851974365] [map_server]: Creating bond (map_server) to lifecycle manager.
[component_container_isolated-10] [INFO] [1778058276.853778316] [amcl]: Received a 1157 X 979 map @ 0.050 m/pix
[robot_pose_tf_bridge.py-11] [WARN] [1778058276.862185835] [robot_pose_tf_bridge]: Cannot lookup transform map -> base_link: "map" passed to lookupTransform argument target_frame does not exist. 
[scan_to_scan_filter_chain-7] [WARN] [1778058276.881658547] [laser_filter]: diagnostic_updater: No HW_ID was set. This is probably a bug. Please report it. For devices that do not have a HW_ID, set this value to 'none'. This warning only occurs once all diagnostics are OK. It is okay to wait until the device is open before calling setHardwareID.
[component_container_isolated-10] [INFO] [1778058276.917453025] [velocity_smoother]: 
[component_container_isolated-10] 	velocity_smoother lifecycle node launched. 
[component_container_isolated-10] 	Waiting on external lifecycle transitions to activate
[component_container_isolated-10] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/velocity_smoother' in container '/nav2_container'
[component_container_isolated-10] [INFO] [1778058276.922880430] [nav2_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_lifecycle_manager::LifecycleManager>
[component_container_isolated-10] [INFO] [1778058276.922963400] [nav2_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<nav2_lifecycle_manager::LifecycleManager>
[spawner-6] [INFO] [1778058276.928485645] [spawner_diff_drive_controller]: Loaded diff_drive_controller
[component_container_isolated-10] [INFO] [1778058276.957267117] [lifecycle_manager_navigation]: Creating
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/lifecycle_manager_navigation' in container '/nav2_container'
[component_container_isolated-10] [INFO] [1778058276.966837742] [lifecycle_manager_navigation]: Creating and initializing lifecycle service clients
[component_container_isolated-10] [INFO] [1778058276.972212064] [lifecycle_manager_localization]: Server map_server connected with bond.
[component_container_isolated-10] [INFO] [1778058276.972309272] [lifecycle_manager_localization]: Activating amcl
[component_container_isolated-10] [INFO] [1778058276.972590977] [amcl]: Activating
[component_container_isolated-10] [INFO] [1778058276.972678682] [amcl]: initialPoseReceived
[component_container_isolated-10] [INFO] [1778058276.972968195] [amcl]: Setting pose (1778058276.972965): 3.060 -0.570 -0.630
[component_container_isolated-10] [INFO] [1778058276.974320631] [amcl]: Creating bond (amcl) to lifecycle manager.
[component_container_isolated-10] [INFO] [1778058277.004716757] [lifecycle_manager_navigation]: Starting managed nodes bringup...
[component_container_isolated-10] [INFO] [1778058277.004843019] [lifecycle_manager_navigation]: Configuring controller_server
[component_container_isolated-10] [INFO] [1778058277.005172176] [controller_server]: Configuring controller interface
[component_container_isolated-10] [INFO] [1778058277.005739491] [controller_server]: getting goal checker plugins..
[component_container_isolated-10] [INFO] [1778058277.006891974] [controller_server]: Controller frequency set to 10.0000Hz
[component_container_isolated-10] [INFO] [1778058277.007014108] [local_costmap.local_costmap]: Configuring
[ros2_control_node-3] [INFO] [1778058277.013613735] [controller_manager]: Configuring controller 'diff_drive_controller'
[component_container_isolated-10] [INFO] [1778058277.024751428] [local_costmap.local_costmap]: Using plugin "obstacle_layer"
[component_container_isolated-10] [INFO] [1778058277.035502912] [local_costmap.local_costmap]: Subscribed to Topics: scan
[component_container_isolated-10] [INFO] [1778058277.051375134] [local_costmap.local_costmap]: Initialized plugin "obstacle_layer"
[component_container_isolated-10] [INFO] [1778058277.051474006] [local_costmap.local_costmap]: Using plugin "inflation_layer"
[component_container_isolated-10] [INFO] [1778058277.055904176] [local_costmap.local_costmap]: Initialized plugin "inflation_layer"
[spawner-6] [INFO] [1778058277.068926453] [spawner_diff_drive_controller]: Configured and activated diff_drive_controller
[component_container_isolated-10] [INFO] [1778058277.080954026] [controller_server]: Created progress_checker : progress_checker of type nav2_controller::SimpleProgressChecker
[component_container_isolated-10] [INFO] [1778058277.083949176] [controller_server]: Created goal checker : general_goal_checker of type nav2_controller::SimpleGoalChecker
[component_container_isolated-10] [INFO] [1778058277.085797955] [controller_server]: Controller Server has general_goal_checker  goal checkers available.
[component_container_isolated-10] [INFO] [1778058277.087192466] [lifecycle_manager_localization]: Server amcl connected with bond.
[component_container_isolated-10] [INFO] [1778058277.087263532] [lifecycle_manager_localization]: Managed nodes are active
[component_container_isolated-10] [INFO] [1778058277.087302761] [lifecycle_manager_localization]: Creating bond timer...
[component_container_isolated-10] [INFO] [1778058277.087330759] [controller_server]: Created controller : FollowPath of type nav2_mppi_controller::MPPIController
[component_container_isolated-10] [INFO] [1778058277.096894563] [controller_server]: Controller period is equal to model dt. Control sequence shifting is ON
[component_container_isolated-10] [INFO] [1778058277.101695839] [controller_server]: ConstraintCritic instantiated with 1 power and 4.000000 weight.
[component_container_isolated-10] [INFO] [1778058277.101896463] [controller_server]: Critic loaded : mppi::critics::ConstraintCritic
[component_container_isolated-10] [WARN] [1778058277.106485628] [controller_server]: Inconsistent configuration in collision checking. Please verify the robot's shape settings in both the costmap and the cost critic.
[component_container_isolated-10] Magick: abort due to signal 11 (SIGSEGV) "Segmentation Fault"...
[INFO] [spawner-5]: process has finished cleanly [pid 23669]
[INFO] [spawner-6]: process has finished cleanly [pid 23671]
[ydlidar_ros2_driver_node-4] [2026-05-06 18:04:38][info] Start to getting intensity flag
[ERROR] [component_container_isolated-10]: process has died [pid 23705, exit code -6, cmd '/opt/ros/humble/lib/rclcpp_components/component_container_isolated --ros-args --log-level info --ros-args -r __node:=nav2_container --params-file /tmp/launch_params_p3dhje0y --params-file /tmp/launch_params_dw22cksw -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped -r /tf:=tf -r /tf_static:=tf_static'].
[ydlidar_ros2_driver_node-4] [2026-05-06 18:04:38][info] Auto set intensity 0
[ydlidar_ros2_driver_node-4] [2026-05-06 18:04:38][info] [YDLIDAR] End to getting intensity flag
[ydlidar_ros2_driver_node-4] [2026-05-06 18:04:38][info] [YDLIDAR] Create thread 0x92C8B8E0
[robot_pose_tf_bridge.py-11] [WARN] [1778058278.961890054] [robot_pose_tf_bridge]: Cannot lookup transform map -> base_link: "map" passed to lookupTransform argument target_frame does not exist. 
[ydlidar_ros2_driver_node-4] [2026-05-06 18:04:39][info] Successed to start scan mode, Elapsed time 2584 ms
[ydlidar_ros2_driver_node-4] [2026-05-06 18:04:39][info] Scan Frequency: 10.00Hz
[ydlidar_ros2_driver_node-4] [2026-05-06 18:04:39][info] Fixed Size: 1830
[ydlidar_ros2_driver_node-4] [2026-05-06 18:04:39][info] Sample Rate: 18.00K
[ydlidar_ros2_driver_node-4] [2026-05-06 18:04:39][info] Successed to check the lidar, Elapsed time 0 ms
[ydlidar_ros2_driver_node-4] [2026-05-06 18:04:39][info] Now lidar is scanning...
[robot_pose_tf_bridge.py-11] [WARN] [1778058280.963009534] [robot_pose_tf_bridge]: Cannot lookup transform map -> base_link: "map" passed to lookupTransform argument target_frame does not exist. 
[robot_pose_tf_bridge.py-11] [WARN] [1778058283.063179373] [robot_pose_tf_bridge]: Cannot lookup transform map -> base_link: "map" passed to lookupTransform argument target_frame does not exist. 
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[ros2_control_node-3] [INFO] [1778058284.339271463] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-3] [INFO] [1778058284.339472022] [controller_manager]: Shutdown request received....
[ros2_control_node-3] [INFO] [1778058284.339588364] [controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-3] [INFO] [1778058284.339635976] [controller_manager]: Deactivating controller 'joint_state_broadcaster'
[ros2_control_node-3] [INFO] [1778058284.339700930] [controller_manager]: Shutting down controller 'joint_state_broadcaster'
[ros2_control_node-3] [INFO] [1778058284.339749566] [controller_manager]: Deactivating controller 'diff_drive_controller'
[ros2_control_node-3] [INFO] [1778058284.339798810] [controller_manager]: Shutting down controller 'diff_drive_controller'
[ros2_control_node-3] [INFO] [1778058284.339844278] [resource_manager]: 'deactivate' hardware 'DiffDriveSystem' 
[ros2_control_node-3] [INFO] [1778058284.339899666] [resource_manager]: Successful 'deactivate' of hardware 'DiffDriveSystem'
[ros2_control_node-3] [INFO] [1778058284.339907857] [resource_manager]: 'shutdown' hardware 'DiffDriveSystem' 
[ros2_control_node-3] [INFO] [1778058284.339915312] [resource_manager]: Successful 'shutdown' of hardware 'DiffDriveSystem'
[ros2_control_node-3] [INFO] [1778058284.339920496] [controller_manager]: Shutting down the controller manager.
[INFO] [scan_to_scan_filter_chain-7]: process has finished cleanly [pid 23673]
[INFO] [robot_state_publisher-2]: process has finished cleanly [pid 23663]
[INFO] [ros2_control_node-3]: process has finished cleanly [pid 23665]
[INFO] [imu_node-8]: process has finished cleanly [pid 23675]
[INFO] [ekf_node-9]: process has finished cleanly [pid 23677]
[ERROR] [robot_pose_tf_bridge.py-11]: process has died [pid 23719, exit code 1, cmd '/home/chan/navigation_stack_lab/ws_robot/install/robot_base/lib/robot_base/robot_pose_tf_bridge.py --ros-args -r __node:=robot_pose_tf_bridge --params-file /tmp/launch_params_333ueo3a -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped'].
[scan_to_scan_filter_chain-7] [INFO] [1778058284.339269127] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[imu_node-8] [INFO] [1778058284.339321891] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ekf_node-9] [INFO] [1778058284.339298725] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ydlidar_ros2_driver_node-4] [INFO] [1778058284.339412571] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ydlidar_ros2_driver_node-4] [INFO] [1778058284.432896609] [ydlidar_ros2_driver_node]: [YDLIDAR INFO] Now YDLIDAR is stopping .......
[robot_state_publisher-2] [INFO] [1778058284.339513138] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ydlidar_ros2_driver_node-4] [2026-05-06 18:04:44][info] Now lidar scanning has stopped!
[robot_pose_tf_bridge.py-11] Traceback (most recent call last):
[robot_pose_tf_bridge.py-11]   File "/home/chan/navigation_stack_lab/ws_robot/install/robot_base/lib/robot_base/robot_pose_tf_bridge.py", line 104, in <module>
[robot_pose_tf_bridge.py-11]     main()
[robot_pose_tf_bridge.py-11]   File "/home/chan/navigation_stack_lab/ws_robot/install/robot_base/lib/robot_base/robot_pose_tf_bridge.py", line 100, in main
[robot_pose_tf_bridge.py-11]     rclpy.shutdown()
[robot_pose_tf_bridge.py-11]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 130, in shutdown
[robot_pose_tf_bridge.py-11]     _shutdown(context=context)
[robot_pose_tf_bridge.py-11]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/utilities.py", line 58, in shutdown
[robot_pose_tf_bridge.py-11]     return context.shutdown()
[robot_pose_tf_bridge.py-11]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/context.py", line 102, in shutdown
[robot_pose_tf_bridge.py-11]     self.__context.shutdown()
[robot_pose_tf_bridge.py-11] rclpy._rclpy_pybind11.RCLError: failed to shutdown: rcl_shutdown already called on the given context, at ./src/rcl/init.c:241
[INFO] [ydlidar_ros2_driver_node-4]: process has finished cleanly [pid 23667]
