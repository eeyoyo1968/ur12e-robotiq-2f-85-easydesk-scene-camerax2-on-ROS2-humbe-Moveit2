import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    description_pkg = "my_ur_description"
    
    # 1. Load URDF
    xacro_file = os.path.join(get_package_share_directory(description_pkg), 'urdf', 'ur_system.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={'use_fake_hardware': 'true'})
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 2. Load SRDF
    srdf_file = os.path.join(get_package_share_directory(description_pkg), 'srdf', 'ur_system.srdf')
    with open(srdf_file, 'r') as f:
        semantic_config = f.read()
    robot_description_semantic = {'robot_description_semantic': semantic_config}

    # 3. Load Paths
    kinematics_yaml = os.path.join(get_package_share_directory(description_pkg), 'config', 'kinematics.yaml')
    controllers_yaml = os.path.join(get_package_share_directory(description_pkg), 'config', 'ur_controllers.yaml')

    # 4. MoveIt Controller Config
    moveit_controllers = {
        'moveit_simple_controller_manager': {
            'controller_names': ['joint_trajectory_controller', 'robotiq_gripper_controller'],
            'joint_trajectory_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
            },
            'robotiq_gripper_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': ['robotiq_85_left_knuckle_joint'],
            },
        }
    }

    # 5. Planning Pipeline Config (FIXED: adapters changed from list to space-separated string)
    planning_pipeline_config = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization ' \
                                'default_planner_request_adapters/FixWorkspaceBounds ' \
                                'default_planner_request_adapters/FixStartStateBounds ' \
                                'default_planner_request_adapters/FixStartStateCollision ' \
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='both', parameters=[robot_description]),

        # ros2_control_node: Explicitly load the controllers yaml here
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controllers_yaml],
            output='both',
        ),

        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                planning_pipeline_config,
                moveit_controllers,
                {'use_sim_time': False, 'publish_planning_scene': True}
            ],
        ),

        # Spawners
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']),
        Node(package='controller_manager', executable='spawner', arguments=['joint_trajectory_controller']),
        Node(package='controller_manager', executable='spawner', arguments=['robotiq_gripper_controller']),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[robot_description, robot_description_semantic, kinematics_yaml],
        )
    ])