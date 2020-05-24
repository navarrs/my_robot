#!/bin/zsh

ROBOT_LIST="dummy"
ROBOT=""

MODES_LIST="goal keyboard"
MODE=""

MAP_POSE="-1.57079632679"
MAPS_LIST="pgm_map rtab_map"
MAP=""

# Checks if a list contains a word
function contains {
  local list="$1"
  local item="$2"
  if [[ $list =~ (^|[[:space:]])"$item"($|[[:space:]]) ]] ; then
    result=0
  else
    result=1
  fi
  return $result
}
# Prints script usage
function usage {
	echo "Usage: ./scripts/robot_spawn.zsh -r robot_name"
	echo "-r: Robot to spawn [dummy]"
	echo "-m: Mode of operation [goal | keyboard]"
}
# If no arguments where provided, print usage and exit
if [ $# -eq 0 ] ; then
  usage
	exit 1
fi
# Capture options
while getopts r:m:f:a: option ; do
	case "${option}" in
	r) 
		ROBOT="$(echo ${OPTARG} | tr '[:upper:]' '[:lower:]')"
		if ! `contains "$ROBOT_LIST" "$ROBOT"` ; then 
			echo "Error: unknown robot. Available robots are: [dummy]"
			exit 1
	  fi
	  ;;
	m) 
		MODE="$(echo ${OPTARG} | tr '[:upper:]' '[:lower:]')"
		if ! `contains "$MODES_LIST" "$MODE"` ; then 
			echo "Error: unknown mode. Available modes are: [goal | keyboard]"
			exit 1
		fi
		;;
	f) 
		MAP="$(echo ${OPTARG} | tr '[:upper:]' '[:lower:]')"
		if ! `contains "$MAPS_LIST" "$MAP"` ; then 
			echo "Error: unknown map. Available maps are: [pgm_map | rtab_map]"
			exit 1
		fi
		;;
	a)
		POSE=$(echo ${OPTARG})
	esac
done
# Launch the world in one terminal 
if  ! [ "$ROBOT" = "" ] ; then
	xterm -e "source catkin_ws/devel/setup.zsh;
	          roslaunch robot world_robot.launch my_robot:=$ROBOT" &
	sleep 15
	# Launch the localization module in another terminal 
	
	xterm -e "source catkin_ws/devel/setup.zsh;
						roslaunch localization amcl.launch map_file:=$MAP initial_pose_a:=$POSE" &
	# Launch the tele-operation package to control the robot with the keyboard
	if [ "$MODE" = "keyboard" ] ; then 
		sleep 2
		xterm -e "source catkin_ws/devel/setup.zsh;
							rosrun teleop_twist_keyboard teleop_twist_keyboard.py" &
	fi
fi