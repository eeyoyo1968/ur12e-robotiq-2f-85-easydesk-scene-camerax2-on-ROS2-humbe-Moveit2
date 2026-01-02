import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    # 1. Define paths (Adjust these to your actual workspace paths)
    # Ensure this file exists at the path below!
    xacro_path = '/ros2_ws/src/my_ur_description/urdf/ur_system.xacro'
    # srdf_path = '/tmp/ur12e.srdf' # Keep for now, we will move it later
    srdf_path = '/ros2_ws/src/my_ur_description/srdf/ur_system.srdf'

    # 2. Process Xacro to generate URDF
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 3. Load SRDF
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # 4. Define MoveGroup Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            '/ros2_ws/install/ur_moveit_config/share/ur_moveit_config/config/moveit_controllers.yaml',
            '/ros2_ws/install/ur_moveit_config/share/ur_moveit_config/config/kinematics.yaml',
            {
                'planning_pipelines': ['ompl'],
                'default_planning_pipeline': 'ompl',
                'ompl.planning_plugin': 'ompl_interface/OMPLPlanner',
                'ompl.request_adapters': (
                    'default_planner_request_adapters/AddTimeOptimalParameterization '
                    'default_planner_request_adapters/FixWorkspaceBounds '
                    'default_planner_request_adapters/FixStartStateBounds '
                    'default_planner_request_adapters/FixStartStateCollision '
                    'default_planner_request_adapters/FixStartStatePathConstraints'
                ),
                'moveit_manage_controllers': True,
                'trajectory_execution.allowed_start_tolerance': 0.1,
                'use_sim_time': False,
            }
        ],
    )

    # 5. Define RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', '/ros2_ws/install/ur_moveit_config/share/ur_moveit_config/rviz/view_robot.rviz'],
        parameters=[
            robot_description,
            robot_description_semantic,
            '/ros2_ws/install/ur_moveit_config/share/ur_moveit_config/config/kinematics.yaml'
        ],
    )

    # 6. Controller auto-activation (5 second delay)
    activate_controller = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'set_controller_state', 'joint_trajectory_controller', 'active'],
                output='screen'
            )
        ]
    )

    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': False}]
    )
    
    return LaunchDescription([
        move_group_node,
        rviz_node,
        activate_controller,
        load_gripper_controller,
        robot_state_publisher
    ])