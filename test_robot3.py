import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time

class Ur12eControl(Node):
    def __init__(self):
        super().__init__('ur12e_script_node')
        self._arm_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/robotiq_gripper_controller/follow_joint_trajectory')

    def send_goal_blocking(self, client, joint_names, positions, duration):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = duration
        goal_msg.trajectory.points = [point]

        client.wait_for_server()
        self.get_logger().info(f'Sending goal to {client._action_name}...')
        
        send_goal_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

def main():
    rclpy.init()
    node = Ur12eControl()
    
    arm_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    gripper_joints = ['robotiq_85_left_knuckle_joint']

    # 1. Move Arm
    print("Moving Arm to Pose...")
    node.send_goal_blocking(node._arm_client, arm_joints, [0.0, -1.57, 1.57, -1.57, -1.57, 0.0], 4)

    # 2. Close Gripper
    print("Closing Gripper...")
    node.send_goal_blocking(node._gripper_client, gripper_joints, [0.8], 2)
    
    # 3. Open Gripper
    print("Opening Gripper...")
    node.send_goal_blocking(node._gripper_client, gripper_joints, [0.0], 2)

    # 4. Return Arm Home
    print("Moving Arm Home...")   
    node.send_goal_blocking(node._arm_client, arm_joints, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 4)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()