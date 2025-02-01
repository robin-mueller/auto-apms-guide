#!/bin/bash

# Check if a workspace path is provided
if [ -z "$1" ]; then
  echo "Usage: $0 <absolute_path_to_ros2_workspace>"
  exit 1
fi

WORKSPACE_ROOT_DIR=$1

# Verify the workspace directory exists
if [ ! -d "$WORKSPACE_ROOT_DIR" ]; then
  echo "The directory $WORKSPACE_ROOT_DIR does not exist. Please provide a valid absolute path to your ROS 2 workspace."
  exit 1
fi

# Check if the workspace has been built
if [ ! -f "$WORKSPACE_ROOT_DIR/install/setup.sh" ]; then
  echo "The ROS 2 workspace has not been built (assumed current working directory: $WORKSPACE_ROOT_DIR)."
  exit 1
fi

# Source the workspace environment
source $WORKSPACE_ROOT_DIR/install/setup.sh

# Check if the required package exists
if ! ros2 pkg list 2>/dev/null | grep -q "^auto_apms_behavior_tree_core$"; then
  echo "The package 'auto_apms_behavior_tree_core' is not found in the workspace."
  echo "Ensure the package is installed to the workspace using 'colcon build'."
  exit 1
fi

# Run the ROS 2 command


echo "Creating full behavior tree node reference using the following command: ros2 run auto_apms_behavior_tree_core create_node_reference_markdown"
ros2 run auto_apms_behavior_tree_core create_node_reference_markdown ./src/reference/create_node_reference_markdown_output.md include_native auto_apms_behavior_tree::behavior_tree_nodes auto_apms_simulation::behavior_tree_nodes auto_apms_px4::behavior_tree_nodes

# Check the result
if [ $? -eq 0 ]; then
  exit 0
else
  echo "Failed to execute the command. Check if the package and its dependencies are correctly installed."
  exit 1
fi
