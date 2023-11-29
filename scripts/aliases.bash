alias rcd='rosbag record car_vel imu_raw scan_raw'
alias up='roslaunch ucar_bringup up.launch'
alias mapping='roslaunch ucar_navigation carto_mapping.launch'
alias nav='roslaunch ucar_navigation nav.launch'
alias tp='roslaunch teleop_ackermann_joy teleop.launch'

map_save () {
	basepath="$(rospack find ucar_navigation)"
	echo $basepath
	# Finish the first trajectory. No further data will be accepted on it.
	rosservice call /finish_trajectory 0

	# Ask Cartographer to serialize its current state.
	# (press tab to quickly expand the parameter syntax)
	rosservice call /write_state "{filename: '$basepath/map/map.pbstream', include_unfinished_submaps: "true"}"

	# Save OccupancyGrid map
	rosrun map_server map_saver -f "$basepath/map/map"
}
