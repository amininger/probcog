#!/bin/bash

# Run in background
BG="FALSE"
# Iterate over each argument given

while [[ $# -gt 0 ]]
do
	arg="$1"
	case $arg in
		-b|--background)
			BG="TRUE"
			;;
		*)
			WORLD_FILE=$1
			;;
	esac
	shift
done

export PROBCOG_CONFIG=$MOBILE_SIM_HOME/config/robot.config.local

if [ "$WORLD_FILE" == "" ]; then 
	echo "Missing world file"
	exit 1
fi

WORLD_FILE=$MOBILE_SIM_HOME/worlds/$WORLD_FILE

cd $MOBILE_SIM_HOME
if [ "$BG" == "TRUE" ]; then
	java soargroup.rosie.mobilesim.MobileGUI -w $WORLD_FILE -s &
else
	java soargroup.rosie.mobilesim.MobileGUI -w $WORLD_FILE -s
fi

