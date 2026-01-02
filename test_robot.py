import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

def main():
    rclpy.init()
    node = rclpy.create_node('gripper_test_node')
    
    # In a real script, you would use MoveIt's MoveGroupInterface
    # For now, you can trigger your named poses via terminal to test:
    node.get_logger().info("SRDF Configured! Use RViz to select group 'gripper' and pose 'open/close'.")
    
    # Example terminal commands to test the new SRDF:
    # ros2 run moveit_ros_visualization moveit_joy.py (if using joystick)
    # Or just use the RViz Motion Planning Panel
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()