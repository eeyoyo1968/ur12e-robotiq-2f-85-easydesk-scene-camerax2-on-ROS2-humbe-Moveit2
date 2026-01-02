import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

# Note: In a full setup, you'd use moveit_py here, 
# but for now, we use the JointTrajectory interface for the arm too.

class UR12eAutomator(Node):
    def __init__(self):
        super().__init__('ur12e_automator')
        self.gripper_pub = self.create_publisher(JointTrajectory, '/robotiq_gripper_controller/joint_trajectory', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

    def send_cmd(self, pub, joints, positions, duration=2):
        msg = JointTrajectory()
        msg.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = duration
        msg.points = [point]
        pub.publish(msg)
        time.sleep(duration + 0.5)

    def run(self):
        arm_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        print("1. Opening Gripper...")
        self.send_cmd(self.gripper_pub, ['robotiq_85_left_knuckle_joint'], [0.0])

        print("2. Moving to Pick Position (Above Desk)...")
        # [pan, lift, elbow, w1, w2, w3] - Adjust these values for your desk height
        self.send_cmd(self.arm_pub, arm_joints, [0.0, -1.2, 1.5, -1.8, -1.57, 0.0])

        print("3. Closing Gripper...")
        self.send_cmd(self.gripper_pub, ['robotiq_85_left_knuckle_joint'], [0.7])

        print("4. Lifting Object...")
        self.send_cmd(self.arm_pub, arm_joints, [0.0, -1.57, 1.57, -1.57, -1.57, 0.0])

if __name__ == '__main__':
    rclpy.init()
    node = UR12eAutomator()
    node.run()
    rclpy.shutdown()