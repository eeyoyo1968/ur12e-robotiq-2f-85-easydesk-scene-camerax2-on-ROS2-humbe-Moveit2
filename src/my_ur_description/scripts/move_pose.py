import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
import sys

class UR12ePoseController(Node):
    def __init__(self):
        super().__init__('ur12e_pose_controller')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Waiting for MoveGroup...')
        self._action_client.wait_for_server()

    def move_to_pose(self, x, y, z):
        """Moves to target using your exact RViz rotation values"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        # 1. Use 'BiTRRT' for 'reasonable' arcs like RViz
        goal_msg.request.planner_id = "BiTRRT"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.max_velocity_scaling_factor = 0.2
        
        # 2. SEED: This makes the solver stay close to current joint values
        goal_msg.request.start_state.is_diff = True

        # 3. CONSTRAINTS
        c = Constraints()

        # --- Position (Target XYZ) ---
        pos = PositionConstraint()
        pos.header.frame_id = "world"
        pos.link_name = "wrist_3_link"
        
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.005] # 5mm tolerance
        
        target_pose = PoseStamped()
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        
        pos.constraint_region.primitives.append(s)
        pos.constraint_region.primitive_poses.append(target_pose.pose)
        c.position_constraints.append(pos)

        # --- Orientation (Your Exact RViz Values) ---
        ori = OrientationConstraint()
        ori.header.frame_id = "world"
        ori.link_name = "wrist_3_link"
        
        # Cleaning the scientific notation values for stability
        ori.orientation.x = -0.70710678
        ori.orientation.y = 0.0
        ori.orientation.z = 0.0
        ori.orientation.w = 0.70710678
        
        # Give the orientation a tiny bit of room (0.01 rad is about 0.5 degrees)
        # This prevents the solver from 'spinning' to find a 0.0000001 precision match
        ori.absolute_x_axis_tolerance = 0.01
        ori.absolute_y_axis_tolerance = 0.01
        ori.absolute_z_axis_tolerance = 0.01
        c.orientation_constraints.append(ori)

        goal_msg.request.goal_constraints.append(c)

        self.get_logger().info(f"Planning to XYZ: {x}, {y}, {z}")
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        handle = send_goal_future.result()
        if handle.accepted:
            res_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            self.get_logger().info("Success!")
        return True

def main():
    rclpy.init()
    bot = UR12ePoseController()
    
    # Test with a known reasonable coordinate
    bot.move_to_pose(0.6, 0.0, 1.1)
    
    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()