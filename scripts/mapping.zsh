#!/bin/zsh

ROBOT_LIST="dummy"
ROBOT=""

# Create db directory 
DB_PATH="./catkin_ws/src/database/slam/"
if [ ! -d $DB_PATH ]; then
  mkdir $DB_PATH
else
	echo "Info: Database directory already exists"
fi 

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
	echo "Usage: ./scripts/rtab_mapping.zsh -r robot_name"
	echo "-r: Robot to spawn [dummy]"
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
			echo "Error: unknown robot. Available robots are: [none | dummy]"
			exit 1
	  fi
	  ;;
esac
done

# Launch mapping 
if ! [ "$ROBOT" = "" ] ; then
	# Launch the world with a specified robot
	xterm -e "source catkin_ws/devel/setup.zsh;
		  	    roslaunch robot world_robot.launch my_robot:=$ROBOT" &
	sleep 10
	# Launch RTAB-Map
	xterm -e "source catkin_ws/devel/setup.zsh;
	         roslaunch mapping mapping.launch" &
	# Launch Teleop
	xterm -e "source catkin_ws/devel/setup.zsh;
						rosrun teleop_twist_keyboard teleop_twist_keyboard.py" &
fi