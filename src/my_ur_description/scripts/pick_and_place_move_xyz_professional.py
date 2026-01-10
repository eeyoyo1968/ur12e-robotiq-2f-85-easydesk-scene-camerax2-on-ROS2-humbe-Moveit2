import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, 
    JointConstraint, 
    PositionConstraint, 
    OrientationConstraint, 
    BoundingVolume
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class UR12eFinalPicker(Node):
    def __init__(self):
        # FIX: Ensure the node name is passed correctly here
        super().__init__('ur12e_final_picker')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info("Waiting for MoveGroup...")
        self._action_client.wait_for_server()

    def move_to_home(self):
        self.get_logger().info("Moving to HOME to satisfy constraints...")
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        js = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        con = Constraints()
        for name, pos in zip(names, js):
            jc = JointConstraint(joint_name=name, position=pos, 
                                 tolerance_above=0.1, tolerance_below=0.1, weight=1.0)
            con.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(con)
        return self._send_goal(goal_msg)

    def move_to_pose(self, x, y, z):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        # 1. SET THE TARGET (Goal Constraints)
        target_constraints = Constraints()
        
        # Position + Orientation (as you had before)
        # ... [Insert your PositionConstraint and OrientationConstraint here] ...
        
        # 2. ADD JOINT LOCK TO THE PATH (Preventing the Flip)
        path_constraints = Constraints()
        
        # Forcing the elbow to stay in the "Up" configuration
        # Assuming 1.57 is your target, we allow it to move but NOT cross 0.0
        elbow_lock = JointConstraint()
        elbow_lock.joint_name = "elbow_joint"
        elbow_lock.position = 1.57  # Target config
        elbow_lock.tolerance_above = 1.5  # Can go up to ~3.0
        elbow_lock.tolerance_below = 1.5  # Can go down to ~0.07 (but not cross 0)
        elbow_lock.weight = 1.0
        
        path_constraints.joint_constraints.append(elbow_lock)
        
        # Optional: Lock the Shoulder to prevent "Front/Back" flips
        shoulder_lock = JointConstraint()
        shoulder_lock.joint_name = "shoulder_lift_joint"
        shoulder_lock.position = -1.57
        shoulder_lock.tolerance_above = 1.5
        shoulder_lock.tolerance_below = 1.5
        shoulder_lock.weight = 1.0
        path_constraints.joint_constraints.append(shoulder_lock)

        goal_msg.request.path_constraints = path_constraints
        
        # 3. CRITICAL: Increase planning time
        # Constrained planning is much harder for the computer
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.num_planning_attempts = 10

        self.get_logger().info(f"Planning move to ({x}, {y}, {z}) with Elbow-Up constraint...")
        return self._send_goal(goal_msg)

    def _send_goal(self, goal_msg):
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted: return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

def main():
    rclpy.init()
    bot = UR12eFinalPicker()
    
    bot.move_to_home()
    time.sleep(5.0)
    targets = [(0.7, 0.0, 1.2), (0.7, 0.1, 1.2), (0.7, -0.1, 1.2)]

    for x, y, z in targets:
        print("target")
        bot.move_to_pose(x, y, z)
        time.sleep(5.0)

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()