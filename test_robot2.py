import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class Ur12eControl(Node):
    def __init__(self):
        super().__init__('ur12e_script_node')
        # Action client for the Arm
        self._arm_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        # Action client for the Gripper (assuming robotiq_gripper_controller is loaded)
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/robotiq_gripper_controller/follow_joint_trajectory')

    def send_arm_goal(self, positions):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 3
        goal_msg.trajectory.points = [point]
        
        self._arm_client.wait_for_server()
        return self._arm_client.send_goal_async(goal_msg)

    def send_gripper_goal(self, gap_value):
        # 0.0 is fully open, 0.8 is fully closed for Robotiq 85
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['robotiq_85_left_knuckle_joint']
        point = JointTrajectoryPoint()
        point.positions = [gap_value]
        point.time_from_start.sec = 2
        goal_msg.trajectory.points = [point]
        
        self._gripper_client.wait_for_server()
        self._gripper_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    node = Ur12eControl()
    
    print("Moving Arm...")
    node.send_arm_goal([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
    print("Moving Arm Home...")   
    node.send_arm_goal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    
    # Small delay or wait for result would go here
    print("Closing Gripper...")
    node.send_gripper_goal(0.8) 

    print("opening Gripper...")
    node.send_gripper_goal(0.0) 
    
    rclpy.spin_once(node)

if __name__ == '__main__':
    main()