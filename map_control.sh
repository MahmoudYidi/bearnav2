#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 {start_mapping|stop_mapping|repeat_map} map_name"
    exit 1
fi

COMMAND=$1
MAP_NAME=$2

# Check the command and run the corresponding ROS 2 action
case "$COMMAND" in
    start_map)
        ros2 action send_goal /bearnav2/mapmaker bearnav2/action/MapMaker "{map_name: '$MAP_NAME', start: true}"
        ;;
    stop_map)
        ros2 action send_goal /bearnav2/mapmaker bearnav2/action/MapMaker "{map_name: '$MAP_NAME', start: false}"
        ;;
    repeat_map)
        ros2 action send_goal /bearnav2/repeater bearnav2/action/MapRepeater "{map_name: '$MAP_NAME'}"
        ;;
    *)
        echo "Unknown command: $COMMAND"
        echo "Usage: $0 {start_mapping|stop_mapping|repeat_map} map_name"
        exit 1
        ;;
esac
