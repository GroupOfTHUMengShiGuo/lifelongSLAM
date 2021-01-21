# lifelongSLAM
Lifelong SLAM in Dynamic and Semi-dynamic Environments

dynamic_NDT package: Mapping and localization based on NDT methods.

visualization package: Visualize NDT and other information.

# Build
Step one: Build message package.
catkin_make -DCATKIN_WHITELIST_PACKAGES="ndt_view_msgs"

Step two: Build other packages.
catkin_make -DCATKIN_WHITELIST_PACKAGES=""

# Run
run the following commands in different terminals.
1. roscore
2. rviz
3. rosrun visualization visualization_node
4. rosrun semi_dynamic_test semi_ndt_test

TODO: Add a roslaunch file.
