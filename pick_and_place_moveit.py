import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        # This is a simplified version using raw publishers for the demo
        self.gripper_pub = self.create_publisher(
            JointTrajectory, 
            '/robotiq_gripper_controller/joint_trajectory', 
            10)

    def move_gripper(self, position):
        msg = JointTrajectory()
        msg.joint_names = ['robotiq_85_left_knuckle_joint']
        point = JointTrajectoryPoint()
        point.positions = [position] # 0.0 is open, 0.8 is closed
        point.time_from_start.sec = 1
        msg.points = [point]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'Moving gripper to {position}')

# To keep it simple for now, we will use RViz to move the arm 
# and this script to toggle the gripper.