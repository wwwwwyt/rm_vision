# colcon build 
cmds=(  
    "ros2 launch rm_vision_bringup ahrs_driver.launch.py"
	"sleep 2 && ros2 launch rm_vision_bringup vision_bringup.launch.py"
	)
for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done