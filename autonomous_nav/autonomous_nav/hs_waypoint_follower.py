import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

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
    
class HSWaypointRunner(Node):
    """Minimal: define a waypoint, execute, wait; then define the next one."""

    JOINT_NAMES: List[str] = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
    PLANNING_GROUP: str = 'tmr_arm'
    ACTION_NAME: str = '/move_action'  # keep identical to your setup

    def __init__(self):
        super().__init__('hs_waypoint_runner')
        self.client = ActionClient(self, MoveGroup, self.ACTION_NAME)
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.client.wait_for_server()
        self.get_logger().info('MoveGroup action server ready.')

    # --- helpers --------------------------------------------------------------
    def _goal_from_joints(self, joints: List[float]) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = self.PLANNING_GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

        cs = Constraints()
        for name, val in zip(self.JOINT_NAMES, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            cs.joint_constraints.append(jc)

        req.goal_constraints.append(cs)
        goal.request = req
        goal.planning_options.plan_only = False  # plan + execute
        return goal

    def _send_and_wait(self, joints: List[float]) -> bool:
        goal = self._goal_from_joints(joints)
        send_fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        handle = send_fut.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Goal rejected.')
            return False
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result()
        return bool(res and res.status == 4)  # 4 = STATUS_SUCCEEDED

    # --- script-style sequence (no arrays, no names) --------------------------
    def run(self):
        # Waypoint 1
        if not self._send_and_wait([-0.3665, 0.6109, 0.8727, 1.1519, 0.0, 0.0]): return
        time.sleep(5.0)

        # Waypoint 2
        if not self._send_and_wait([0.0, -1.2217, 1.3788, 1.5359, 0.0, 0.0]): return
        time.sleep(5)

        self.get_logger().info('Sequence complete ✅')

def main(args=None):
    rclpy.init(args=args)

    # Create both nodes
    node = rclpy.create_node('hs_waypoint_follower_nav2pose')
    node2 = HSWaypointRunner()  # MoveGroup controller node

    # Create ActionClient for Nav2
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    # --- Helper: navigation function ---
    def send_and_wait(pose: PoseStamped) -> bool:
        node.get_logger().info('Waiting for Nav2 action server...')
        client.wait_for_server()
        pose.header.stamp = node.get_clock().now().to_msg()

        goal = NavigateToPose.Goal()
        goal.pose = pose

        def feedback_cb(fb):
            try:
                dist = fb.feedback.distance_remaining
                node.get_logger().info(f'Distance remaining: {dist:.2f} m')
            except Exception:
                pass

        send_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(node, send_future)
        handle = send_future.result()

        if not handle or not handle.accepted:
            node.get_logger().error('Goal was rejected!')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result()

        if result is None:
            node.get_logger().error('No result returned.')
            return False

        node.get_logger().info('Goal reached successfully!')
        return True

    # --- Waypoints ---
    wp1 = make_pose(2.77, -0.465, 0.0)
    wp2 = make_pose(5.36, 2.98, -0.7)
    wp3 = make_pose(13.4, 0.906, 0.0)
    wp4 = make_pose(28.1, 5.18, -0.8)
    wp5 = make_pose(30.6, -4.35, -1.6)

    try:
        # --- Go to waypoint 1 ---
        send_and_wait(wp1)
        node.get_logger().info('Waiting at waypoint 1...')
        time.sleep(5.0)

        # --- Run arm sequence between wp1 and wp2 ---
        node.get_logger().info('Running arm sequence before moving to waypoint 2...')
        node2.run()  # executes MoveGroup sequence (arm poses)

        # --- Continue navigation ---
        send_and_wait(wp2)
        node.get_logger().info('Waiting at waypoint 2...')
        time.sleep(1.0)

        send_and_wait(wp3)
        node.get_logger().info('Waiting at waypoint 3...')
        time.sleep(1.0)

        send_and_wait(wp4)
        node.get_logger().info('Waiting at waypoint 4...')
        time.sleep(1.0)

        send_and_wait(wp5)
        node.get_logger().info('Waiting at waypoint 5...')
        time.sleep(5.0)
        
	# Last pose
        node.get_logger().info('Movinga arm to final pose...')
        node2._send_and_wait([0.0, 0.8203, 0.5585, -0.3316, 0.0, 0.0])
        time.sleep(2.0)

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')

    finally:
        node.get_logger().info('Shutting down...')
        node.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()

        
if __name__ == '__main__':
    main()
