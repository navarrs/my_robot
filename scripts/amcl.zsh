#!/bin/zsh

ROBOT_LIST="dummy"
ROBOT=""
WORLD_LIST="house walls"
WORLD=""
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
	echo "Usage: ./scripts/mapping.zsh -r robot_name -w world_name -m mode"
	echo "-r: dummy"
	echo "-w: house | wall"
	echo "-m: goal | keyboard"
}
# If no arguments where provided, print usage and exit
if [ $# -eq 0 ] ; then
  usage
	exit 1
fi
# Capture options
while getopts h:r:w:m:a: option ; do
	case "${option}" in
		h)
		usage
		exit 1
		;;
	r) 
		ROBOT="$(echo ${OPTARG} | tr '[:upper:]' '[:lower:]')"
		if [ "$ROBOT" = "" ] ; then
			echo "Error: specified no robot. Options are: [dummy]"
			exit 1
		fi

		if ! `contains "$ROBOT_LIST" "$ROBOT"` ; then 
			echo "Error: unknown robot. Available robots are: [dummy]"
			exit 1
	  	fi
	  ;;
	w) 
		WORLD="$(echo ${OPTARG} | tr '[:upper:]' '[:lower:]')"
		if [ "$WORLD" = "" ] ; then
			echo "Error: specified no world. Options are: [house|walls]"
			exit 1
		fi
		if ! `contains "$WORLD_LIST" "$WORLD"` ; then 
			echo "Error: unknown world. Available worlds are: [house|walls]"
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
	a)
		POSE=$(echo ${OPTARG})
	esac
done

# Spawn world and robot
xterm -e "source catkin_ws/devel/setup.zsh;
		  roslaunch robot spawn.launch robot_name:=$ROBOT world_name:=$WORLD rviz_file:=localization.rviz" &
sleep 10

# Launch the localization module in another terminal 
xterm -e "source catkin_ws/devel/setup.zsh;
		  roslaunch localization amcl.launch map_name:=$WORLD initial_pose_a:=$POSE" &
	
# Launch the tele-operation package to control the robot with the keyboard
if [ "$MODE" = "keyboard" ] ; then 
	sleep 2
	xterm -e "source catkin_ws/devel/setup.zsh;
			  rosrun teleop_twist_keyboard teleop_twist_keyboard.py" &
fi
