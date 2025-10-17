import math
import time
import re
from typing import List
import json
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint


# --- Helper: build PoseStamped for Nav2 ---
def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.pose.position.x = x
    ps.pose.position.y = y

    half = yaw * 0.5
    ps.pose.orientation.z = math.sin(half)
    ps.pose.orientation.w = math.cos(half)
    return ps


# --- MoveIt arm controller + HMI subscriber node ---
class HSWaypointRunner(Node):
    JOINT_NAMES: List[str] = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    PLANNING_GROUP: str = 'tmr_arm'
    ACTION_NAME: str = '/move_action'

    def __init__(self):
        super().__init__('hs_waypoint_runner')
        # MoveIt action client
        self.client = ActionClient(self, MoveGroup, self.ACTION_NAME)
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.client.wait_for_server()
        self.get_logger().info('MoveGroup action server ready.')

        # Subscribe to HMI topic
        self.last_hmi_msg = None
        self.create_subscription(String, '/hmi/unified_status', self.hmi_callback, 10)

    # --- Callback for HMI messages ---
    def hmi_callback(self, msg):
        self.last_hmi_msg = msg

    # --- Get box weight from last received HMI message ---
    def get_box_weight(self):
        if self.last_hmi_msg is None:
            self.get_logger().warn('No HMI message received yet.')
            return None

        try:
            data = json.loads(self.last_hmi_msg.data)
            weight_str = data.get("box", {}).get("weight_raw", "")
            weight_str = re.sub(r'^[^\d]*', '', weight_str)  # remove leading ": " or junk
            match = re.search(r'[\d\.]+', weight_str)
            if match:
                weight = float(match.group())
                self.get_logger().info(f'Received weight: {weight} kg')
                return weight
            else:
                self.get_logger().error(f'No numeric weight in "{weight_str}"')
                return None
        except Exception as e:
            self.get_logger().error(f'Failed to parse HMI message: {e}')
            return None

    # --- MoveIt helpers ---
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
        goal.planning_options.plan_only = False
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
        return bool(res and res.status == 4)  # STATUS_SUCCEEDED


def main(args=None):
    rclpy.init(args=args)

    # Nodes
    nav_node = rclpy.create_node('hs_waypoint_follower_nav2pose')
    arm_node = HSWaypointRunner()
    client = ActionClient(nav_node, NavigateToPose, 'navigate_to_pose')

    # Arm poses
    POSE_BIG_BOX = [-0.3665, 0.6109, 0.8727, 1.1519, 0.0, 0.0]
    POSE_MED_BOX = [0.4712, 0.3665, 1.3613, 1.5358, 0.0, 0.0]
    POSE_SMALL_BOX = [1.1170, 0.8552, 0.8377, 1.5358, 0.0, 0.0]

    # Waypoints forward
    wp1 = make_pose(2.77, -0.465, 0.0)
    wp2 = make_pose(5.36, 2.98, -0.7)
    wp3 = make_pose(13.4, 0.906, 0.0)
    wp4 = make_pose(28.1, 5.18, -0.8)
    wp5 = make_pose(30.6, -4.35, -1.6)

    # Waypoints backward (opposite orientation)
    wp5_return = make_pose(30.6, -4.35, 1.54)
    wp4_return = make_pose(28.1, 5.18, 2.34)
    wp3_return = make_pose(13.4, 0.906, 3.14)
    wp2_return = make_pose(5.36, 2.98, 2.44)

    # Helper: send nav goal
    def send_and_wait(pose: PoseStamped) -> bool:
        nav_node.get_logger().info('Waiting for Nav2 action server...')
        client.wait_for_server()
        pose.header.stamp = nav_node.get_clock().now().to_msg()

        goal = NavigateToPose.Goal()
        goal.pose = pose

        def feedback_cb(fb):
            try:
                dist = fb.feedback.distance_remaining
                nav_node.get_logger().info(f'Distance remaining: {dist:.2f} m')
            except Exception:
                pass

        send_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(nav_node, send_future)
        handle = send_future.result()

        if not handle or not handle.accepted:
            nav_node.get_logger().error('Goal was rejected!')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(nav_node, result_future)
        result = result_future.result()

        if result is None:
            nav_node.get_logger().error('No result returned.')
            return False

        nav_node.get_logger().info('Goal reached successfully!')
        return True

    try:
        while True:  # Repeat indefinitely
            # Go to waypoint 1 (pick-up location)
            send_and_wait(wp1)
            nav_node.get_logger().info('Waiting at waypoint 1...')
            time.sleep(3.0)

            # --- Reset HMI message and wait for new box ---
            arm_node.last_hmi_msg = None  # Reset to ignore old messages
            timeout = 10.0
            waited = 0.0
            interval = 0.1
            while arm_node.last_hmi_msg is None and waited < timeout:
                rclpy.spin_once(arm_node, timeout_sec=interval)
                waited += interval

            # --- Read box weight ---
            weight = arm_node.get_box_weight()
            if weight is None:
                nav_node.get_logger().warn('No weight message received.')
                chosen_pose = POSE_MED_BOX
            elif weight >= 10.0:
                nav_node.get_logger().info(f'Detected BIG box ({weight:.2f} kg).')
                chosen_pose = POSE_BIG_BOX
            elif weight >= 5.0:
                nav_node.get_logger().info(f'Detected MEDIUM box ({weight:.2f} kg).')
                chosen_pose = POSE_MED_BOX
            else:
                nav_node.get_logger().info(f'Detected SMALL box ({weight:.2f} kg).')
                chosen_pose = POSE_SMALL_BOX

            # Move arm to pickup pose
            nav_node.get_logger().info('Moving arm to pickup pose...')
            arm_node._send_and_wait(chosen_pose)
            time.sleep(2.0)

            # Move arm to safe pose before continuing navigation
            nav_node.get_logger().info('Moving arm to safe pose...')
            arm_node._send_and_wait([0.0, -1.2217, 1.3788, 1.5359, 0.0, 0.0])
            time.sleep(2.0)

            # Continue navigation through waypoints
            for i, wp in enumerate([wp2, wp3, wp4, wp5], start=2):
                send_and_wait(wp)
                nav_node.get_logger().info(f'Waiting at waypoint {i}...')
                time.sleep(1.0)

            # Final arm poses after finishing path
            nav_node.get_logger().info('Moving arm to final pose 1...')
            arm_node._send_and_wait([0.0, 0.8203, 0.5585, -0.3316, 0.0, 0.0])
            time.sleep(1.0)

            nav_node.get_logger().info('Moving arm to safe pose...')
            arm_node._send_and_wait([0.0, -1.2217, 1.3788, 1.5359, 0.0, 0.0])
            time.sleep(2.0)

            # Go back through waypoints in reverse
            for i, wp in enumerate([wp5_return, wp4_return, wp3_return, wp2_return], start=5):
                send_and_wait(wp)
                nav_node.get_logger().info(f'Returning, waiting at waypoint {i}...')
                time.sleep(1.0)

            nav_node.get_logger().info('Returned to start, ready for next box.')

    except KeyboardInterrupt:
        nav_node.get_logger().info('Interrupted by user.')

    finally:
        nav_node.get_logger().info('Shutting down...')
        nav_node.destroy_node()
        arm_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
