#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


# --- Helper function to build a PoseStamped ---
def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    """
    Create a PoseStamped (position + orientation) in the 'map' frame.
    - x, y are coordinates in meters
    - yaw is robot orientation (heading) in radians
    """
    ps = PoseStamped()
    ps.header.frame_id = 'map'  # always use 'map' for navigation goals
    ps.pose.position.x = x
    ps.pose.position.y = y

    # Convert yaw (in radians) into quaternion (needed by ROS2)
    half = yaw * 0.5
    ps.pose.orientation.z = math.sin(half)
    ps.pose.orientation.w = math.cos(half)
    return ps


def main():
    # 1. Initialise ROS2 and create a node
    rclpy.init()
    node = rclpy.create_node('hs_waypoint_follower_nav2pose')

    # 2. Create an ActionClient for the NavigateToPose action
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    # --- Function to send a goal and wait for result ---
    def send_and_wait(pose: PoseStamped) -> bool:
        node.get_logger().info('Waiting for Nav2 action server...')
        client.wait_for_server()

        # Update timestamp (required in headers)
        pose.header.stamp = node.get_clock().now().to_msg()

        # Wrap pose in a NavigateToPose goal message
        goal = NavigateToPose.Goal()
        goal.pose = pose

        # Simple feedback callback: prints distance left to target
        def feedback_cb(fb):
            try:
                dist = fb.feedback.distance_remaining
                node.get_logger().info(f'Distance remaining: {dist:.2f} m')
            except Exception:
                pass  # ignore if feedback doesn't have distance

        # Send the goal
        send_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(node, send_future)
        handle = send_future.result()

        if not handle or not handle.accepted:
            node.get_logger().error('Goal was rejected!')
            return False

        # Wait until navigation is finished
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result()

        if result is None:
            node.get_logger().error('No result returned.')
            return False

        node.get_logger().info('Goal reached successfully!')
        return True

    # --- Hard-coded waypoints for this lab. Edit this section in the code and make it your own ---

    wp1 = make_pose(2.77, -0.465, 0.0)          # Sample goal pose. Make your own! You can create multiple way points
    wp2 = make_pose(5.36, 2.98, -0.7)
    wp3 = make_pose(13.4, 0.906, 0.0)
    wp4 = make_pose(28.1, 5.18, -0.8)
    wp5 = make_pose(30.6, -4.35, -1.6)
    #wp6 = make_pose(9.63, 0.88, 0.0)
    #wp7 = make_pose(7.55, 2.98, 0.0)
    #wp8 = make_pose(5.41, 5.55, 0.0)
    #wp9 = make_pose(1.57, 4.81, 0.0)
    #wp10 = make_pose(1.32, 1.15, 0.0)
    
    # 3. Go to first waypoint
    send_and_wait(wp1)
    # 4. Pause to simulate "waiting at waypoint"
    node.get_logger().info(f'Waiting at waypoint 1...')
    # Add some sort of delay functionality. Look up pythons time module
    time.sleep(5.0)
    # 5. Go to other waypoints and delays if you want
    
    send_and_wait(wp2)
    node.get_logger().info(f'Waiting at waypoint 2...')
    time.sleep(5.0)
    
    send_and_wait(wp3)
    node.get_logger().info(f'Waiting at waypoint 3...')
    time.sleep(5.0)
    
    
    send_and_wait(wp4)
    node.get_logger().info(f'Waiting seconds at waypoint 4...')
    time.sleep(5.0)
    
    send_and_wait(wp5)
    node.get_logger().info(f'Waiting seconds at waypoint 5...')
    time.sleep(1.0)
    
    """
    send_and_wait(wp6)
    node.get_logger().info(f'Waiting {wait_seconds:.0f} seconds at waypoint 6...')
    time.sleep(1.0)
    
    send_and_wait(wp7)
    node.get_logger().info(f'Waiting {wait_seconds:.0f} seconds at waypoint 7...')
    time.sleep(1.0)
    
    send_and_wait(wp8)
    node.get_logger().info(f'Waiting {wait_seconds:.0f} seconds at waypoint 8...')
    time.sleep(1.0)
    
    send_and_wait(wp9)
    node.get_logger().info(f'Waiting {wait_seconds:.0f} seconds at waypoint 9...')
    time.sleep(1.0)
    
    send_and_wait(wp10)
    node.get_logger().info(f'Waiting {wait_seconds:.0f} seconds at waypoint 10...')
    time.sleep(1.0)
    """
   # --- Your custom code ends here ---

    # 6. Shutdown node and ROS2
    node.get_logger().info('Navigation sequence complete. Shutting down.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
