import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time

class UR12ePickAndPlace(Node):
    def __init__(self):
        super().__init__('ur12e_pick_place_node')
        # Action Clients
        self._arm_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/robotiq_gripper_controller/follow_joint_trajectory')

        # Joint Names
        self.arm_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.gripper_joints = ['robotiq_85_left_knuckle_joint']

    def send_command(self, client, joints, positions, duration):
        """Generic function to send a trajectory goal and wait for completion."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joints
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = duration
        goal_msg.trajectory.points = [point]

        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Server {client._action_name} not available!')
            return False

        send_goal_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

def main():
    rclpy.init()
    node = UR12ePickAndPlace()

    # Joint positions (radians)
    home_pose      = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    pick_approach  = [0.5, -1.0, 1.2, -1.7, -1.57, 0.0]
    pick_pose      = [0.5, -1.2, 1.6, -1.9, -1.57, 0.0]
    place_pose     = [-0.5, -1.2, 1.6, -1.9, -1.57, 0.0]
    
    gripper_open   = [0.0]
    gripper_closed = [0.6] # 0.8 is fully closed, 0.6 is good for gripping an object

    try:
        print("1. Moving to Home...")
        node.send_command(node._arm_client, node.arm_joints, home_pose, 4)

        print("2. Opening Gripper...")
        node.send_command(node._gripper_client, node.gripper_joints, gripper_open, 2)

        print("3. Moving to Pick Approach...")
        node.send_command(node._arm_client, node.arm_joints, pick_approach, 3)

        print("4. Moving to Pick Pose...")
        node.send_command(node._arm_client, node.arm_joints, pick_pose, 2)

        print("5. Closing Gripper (Picking Object)...")
        node.send_command(node._gripper_client, node.gripper_joints, gripper_closed, 2)
        time.sleep(1.0) # Ensure grip is firm

        print("6. Lifting Up...")
        node.send_command(node._arm_client, node.arm_joints, pick_approach, 2)

        print("7. Moving to Place Position...")
        node.send_command(node._arm_client, node.arm_joints, place_pose, 3)

        print("8. Opening Gripper (Releasing Object)...")
        node.send_command(node._gripper_client, node.gripper_joints, gripper_open, 2)

        print("9. Returning Home...")
        node.send_command(node._arm_client, node.arm_joints, home_pose, 4)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()