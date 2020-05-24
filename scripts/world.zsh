#!/bin/zsh

ROBOT_LIST="none dummy"
ROBOT=""

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
	echo "-r: Robot to spawn [dummy | ]"
}
# If no arguments where provided, print usage and exit
if [ $# -eq 0 ] ; then
  	usage
		exit 1
fi
# Capture options
while getopts h:r: option ; do
case "${option}" in
	h)
		usage
		exit 1
		;;
	r) 
		ROBOT="$(echo ${OPTARG} | tr '[:upper:]' '[:lower:]')"
		if ! `contains "$ROBOT_LIST" "$ROBOT"` ; then 
			echo "Error: unknown robot. Available robots are: [none | dummy | ]"
			exit 1
	  fi
	  ;;
esac
done
# Launch the world without a robot
if [ "$ROBOT" = "none" ] ; then
  xterm -e "source catkin_ws/devel/setup.zsh;
            roslaunch robot world.launch" &
fi
# Launch the world with a specified robot
if ! [ "$ROBOT" = "" ] ; then
	xterm -e "source catkin_ws/devel/setup.zsh;
		  	    roslaunch robot world_robot.launch my_robot:=$ROBOT" &
fi