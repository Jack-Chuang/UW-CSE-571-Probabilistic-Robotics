#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
# dt-exec echo "This is an empty launch script. Update it to launch your application."
# roscore &
# sleep 5
# dt-exec rosrun my_package static_at_tf_publisher.py
# dt-exec rosrun my_package my_subscriber_node.py
# dt-exec roslaunch my_package multiple_nodes.launch veh:=$VEHICLE_NAME
dt-exec roslaunch sensor_fusion_localization sensor_fusion_node.launch veh:=$VEHICLE_NAME
dt-exec roslaunch my_viz viz_node.launch veh:=$VEHICLE_NAME
dt-exec roslaunch my_viz trace_node.launch veh:=$VEHICLE_NAME
dt-exec roslaunch my_static_tf static_at_tf_publisher.launch veh:=$VEHICLE_NAME
dt-exec roslaunch my_at_localization my_at_pose_node.launch veh:=$VEHICLE_NAME
#dt-exec roslaunch my_viz viz_node.launch veh:=$VEHICLE_NAME



# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
