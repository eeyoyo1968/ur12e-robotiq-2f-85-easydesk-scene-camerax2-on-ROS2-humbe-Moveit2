#!/bin/bash
# -------------------------------------------------------------------
# UR12e + Robotiq 85 Startup Automation Script
# -------------------------------------------------------------------

# 1. Source the workspace
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "Starting UR12e Driver..."

# 2. Launch the UR Driver in the background
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur12e \
  robot_ip:=172.17.0.2 \
  description_package:=ur_workcell \
  description_file:=ur12e_workcell.urdf.xacro \
  controllers_params_file:=/ros2_ws/src/ur_workcell/config/ur12e_controllers.yaml \
  launch_rviz:=true &

# 3. Wait for the controller manager to be ready
echo "Waiting for controller_manager (10s)..."
sleep 10

# 4. Force-load the Gripper Controller
echo "Spawning Robotiq Gripper Controller..."
ros2 run controller_manager spawner robotiq_gripper_controller \
  --controller-type joint_trajectory_controller/JointTrajectoryController \
  --param-file /ros2_ws/src/ur_workcell/config/ur12e_controllers.yaml

echo "Done! Robot is ready for Python scripts."

# Keep the script alive so the background driver doesn't die
wait