import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Paths
    controllers_yaml = "/ros2_ws/install/ur_moveit_config/share/ur_moveit_config/config/moveit_controllers.yaml"
    kinematics_yaml = "/ros2_ws/install/ur_moveit_config/share/ur_moveit_config/config/kinematics.yaml"
    
    # 2. Get Robot Description (URDF) and Semantic Description (SRDF)
    # This pulls from the existing install to ensure RViz knows what the robot looks like
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
        " ur_type:=ur12e"
    ])
    robot_description = {"robot_description": robot_description_content}

    # 3. The MoveGroup Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            controllers_yaml,
            kinematics_yaml,
            {"use_sim_time": False}
        ],
    )

    # 4. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[robot_description, kinematics_yaml]
    )

    return LaunchDescription([move_group_node, rviz_node])