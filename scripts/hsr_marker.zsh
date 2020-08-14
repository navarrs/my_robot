#!/bin/zsh
ROBOT_LIST="dummy"
ROBOT=""
WORLD_LIST="house walls empty"
WORLD=""
RVIZ_CFG="hsr.rviz"
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
	echo "Usage: ./scripts/spawn.zsh -r robot_name -w world_name -v rviz_cfg"
	echo "-r: dummy"
	echo "-w: house | wall | empty"
	echo "-v RVIZ configuration"
}
# If no arguments where provided, print usage and exit
if [ $# -eq 0 ] ; then
  	usage
		exit 1
fi
# Capture options
while getopts h:r:w:v: option ; do
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
			echo "Error: specified no world. Options are: [empty|house|walls]"
			exit 1
		fi
		if ! `contains "$WORLD_LIST" "$WORLD"` ; then 
			echo "Error: unknown world. Available worlds are: [house|walls|empty]"
			exit 1
	  	fi
	  	;;
	v)
		RVIZ_CFG="$(echo ${OPTARG} | tr '[:upper:]' '[:lower:]')"
		if [ "$RVIZ_CFG" = "" ] ; then
			echo "RVIZ Configuration file is unspecified. Setting default."
			RVIZ_CFG="simple.rviz"
		fi
esac
done

# Launch the world without a robot
xterm -e "source catkin_ws/devel/setup.zsh;
		      roslaunch robot spawn.launch robot_name:=$ROBOT world_name:=$WORLD rviz_file:=$RVIZ_CFG" &
sleep 5

# Launch the localization module in another terminal 
xterm -e "source catkin_ws/devel/setup.zsh;
		      roslaunch localization amcl.launch map_name:=$WORLD initial_pose_a:=$POSE" &
sleep 5

xterm -e "source catkin_ws/devel/setup.zsh;
		      roslaunch hsr marker_publisher.launch" &
sleep 5

xterm -e "source catkin_ws/devel/setup.zsh;
		      roslaunch hsr marker_command.launch marker_action:=ADD" &
sleep 5

xterm -e "source catkin_ws/devel/setup.zsh;
		      roslaunch hsr marker_command.launch marker_action:=DELETE" &