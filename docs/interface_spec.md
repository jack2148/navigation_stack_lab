
Interface Spec (Topic / TF Contract)
TF Frames (fixed names)

map

odom

base_footprint

base_link

laser

imu_link

Topics (draft)
Topic	Type	Producer	Consumer	Notes
/scan	sensor_msgs/LaserScan	LiDAR driver	SLAM/Costmap	Hz=?, frame_id=?
/imu/data	sensor_msgs/Imu	IMU driver	EKF	orientation? gyro?
/enc_left	std_msgs/Int64 (draft)	Encoder node	Odom node	cumulative ticks
/enc_right	std_msgs/Int64 (draft)	Encoder node	Odom node	cumulative ticks
/odom	nav_msgs/Odometry	Odom node	SLAM/Nav2	TF: odom->base_footprint
/cmd_vel	geometry_msgs/Twist	Nav2/teleop	Motor driver	v [m/s], w [rad/s]
EOF				

cat > docs/setup.md << 'EOF'

Setup
Workspace

ws/ is a colcon workspace.

Put ROS2 packages under ws/src/.

Build
cd ws
colcon build --symlink-install
source install/setup.bash

Run order (typical)

Motor driver (always-on)

Sensors (/scan, /imu, encoders)

Odom + TF

Mapping OR Localization

Nav2
