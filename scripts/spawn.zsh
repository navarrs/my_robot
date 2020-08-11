#!/bin/zsh
ROBOT_LIST="dummy"
ROBOT=""
WORLD_LIST="house walls empty"
WORLD=""
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
	echo "Usage: ./scripts/spawn.zsh -r robot_name -w world_name"
	echo "-r: dummy"
	echo "-w: house | wall | empty"
}
# If no arguments where provided, print usage and exit
if [ $# -eq 0 ] ; then
  	usage
		exit 1
fi
# Capture options
while getopts h:r:w: option ; do
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
esac
done

# Launch the world without a robot
xterm -e "source catkin_ws/devel/setup.zsh;
		  roslaunch robot spawn.launch robot_name:=$ROBOT world_name:=$WORLD" &