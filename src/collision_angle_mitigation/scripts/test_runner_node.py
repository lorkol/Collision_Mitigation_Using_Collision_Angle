#!/usr/bin/env python3

# =============================================================================
# test_runner_node.py
#
# SEQUENCE:
#   1. Wait for bt_navigator lifecycle state == 'active'
#   2. Publish /initialpose so AMCL knows where the robot is
#      (must match the spawn pose in test_configuration.json)
#   3. Send the navigation goal — Nav2 takes over from here
#   4. Wait obstacle_spawn_delay seconds (configured in JSON)
#   5. Spawn the obstacle in Gazebo
#
# THREADING MODEL:
#   Main thread : rclpy.spin() — processes all callbacks.
#   Sequence thread (daemon): blocking startup sequence.
#     Uses time.sleep() for waiting, future.done() polling for service calls.
#     NEVER calls rclpy.spin*().
#   All subscriptions and service clients are created in __init__.
#   threading.Event is used to signal the sequence thread from spin callbacks.
# =============================================================================

import json
import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory


_QOS_TRANSIENT_LOCAL = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

_QOS_DEFAULT = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)


class TestRunnerNode(Node):

    def __init__(self):
        super().__init__('test_runner_node')

        self.declare_parameter('test_config_file', '')
        config_path = self.get_parameter('test_config_file').value

        if not config_path:
            self.get_logger().fatal(
                'Parameter "test_config_file" is empty. '
                'Pass via: --ros-args -p test_config_file:=/path/to/config.json'
            )
            raise RuntimeError('test_config_file parameter not set')

        if not os.path.exists(config_path):
            self.get_logger().fatal(f'test_config_file not found: {config_path}')
            raise RuntimeError(f'Config file not found: {config_path}')

        with open(config_path, 'r') as f:
            self._config = json.load(f)

        self.get_logger().info(f'Loaded test config: {config_path}')
        self._pkg_share = get_package_share_directory('collision_angle_mitigation')

        # Set by _amcl_callback when /amcl_pose is received
        self._amcl_event = threading.Event()

        # Publishers
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', _QOS_DEFAULT
        )
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', _QOS_DEFAULT)

        # Subscriptions (all created here — never from a thread)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_callback,
            _QOS_TRANSIENT_LOCAL,
        )

        # Service client
        self._bt_nav_state_client = self.create_client(
            GetState, '/bt_navigator/get_state'
        )

        self._sequence_thread = threading.Thread(
            target=self._run_sequence_safe, daemon=True
        )
        self._sequence_thread.start()
        self.get_logger().info('TestRunnerNode initialized. Sequence thread started.')

    # =========================================================================
    # Sequence
    # =========================================================================

    def _run_sequence_safe(self):
        try:
            self._run_sequence()
        except Exception as exc:
            self.get_logger().error(
                f'[Sequence] Unhandled exception — sequence aborted:\n'
                f'  {type(exc).__name__}: {exc}'
            )

    def _run_sequence(self):
        time.sleep(0.5)
        self.get_logger().info('=== Test sequence starting ===')

        # Step 1: Tell AMCL where the robot is.
        # AMCL is managed by lifecycle_manager_localization and becomes active
        # within ~100ms — independently of bt_navigator / controller_server.
        # Publishing here emulates the RViz "2D Pose Estimate" button.
        self._publish_initial_pose()

        # Step 2: Wait until the full Nav2 navigation stack is active.
        # With AMCL now providing map→odom, controller_server can activate.
        self._wait_for_nav2_active()

        # Step 3: Send the navigation goal — Nav2 handles everything from here
        self._send_goal()

        # Step 4: Wait configured delay, then spawn obstacle
        delay = float(self._config.get('obstacle_spawn_delay', 1.0))
        self.get_logger().info(
            f'=== Goal sent. Waiting {delay:.1f}s before spawning obstacle... ==='
        )
        time.sleep(delay)
        self._spawn_obstacle()

        self.get_logger().info('=== Test sequence complete ===')

    # =========================================================================
    # Step 1 — Wait for bt_navigator active
    # =========================================================================

    def _wait_for_nav2_active(self):
        self.get_logger().info('[Step 2] Waiting for bt_navigator to become active...')

        while rclpy.ok():
            if self._bt_nav_state_client.wait_for_service(timeout_sec=1.0):
                break
            self.get_logger().info('  /bt_navigator/get_state not available yet...')

        req = GetState.Request()
        while rclpy.ok():
            future = self._bt_nav_state_client.call_async(req)

            deadline = time.time() + 5.0
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)

            if future.done() and future.result() is not None:
                label = future.result().current_state.label
                if label == 'active':
                    self.get_logger().info('[Step 2] bt_navigator is active. Nav2 ready.')
                    return
                self.get_logger().info(
                    f'  bt_navigator state: "{label}" — waiting for "active"...'
                )
            else:
                self.get_logger().warn('  GetState call timed out.')

            time.sleep(2.0)

    # =========================================================================
    # Step 2 — Set AMCL initial pose
    # =========================================================================

    def _publish_initial_pose(self):
        pose_cfg = self._config['robot_initial_pose']
        x   = float(pose_cfg['x'])
        y   = float(pose_cfg['y'])
        yaw = float(pose_cfg.get('yaw', 0.0))

        self.get_logger().info(
            f'[Step 1] Setting AMCL initial pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.3f} rad'
        )

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.pose.covariance[0]  = 0.25  # x variance
        msg.pose.covariance[7]  = 0.25  # y variance
        msg.pose.covariance[35] = 0.06  # yaw variance

        self._amcl_event.clear()
        deadline = time.time() + 30.0
        while not self._amcl_event.is_set() and time.time() < deadline:
            self._initialpose_pub.publish(msg)
            self._amcl_event.wait(timeout=1.0)

        if self._amcl_event.is_set():
            self.get_logger().info('[Step 1] AMCL confirmed initial pose.')
        else:
            self.get_logger().warn(
                '[Step 1] AMCL did not confirm within 30s — proceeding anyway.'
            )

    def _amcl_callback(self, _msg: PoseWithCovarianceStamped):
        self._amcl_event.set()

    # =========================================================================
    # Step 3 — Send navigation goal
    # =========================================================================

    def _send_goal(self):
        goal_cfg = self._config['goal_pose']
        yaw = float(goal_cfg.get('yaw', 0.0))

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position.x = float(goal_cfg['x'])
        msg.pose.position.y = float(goal_cfg['y'])
        msg.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self._goal_pub.publish(msg)
        self.get_logger().info(
            f'[Step 3] Goal sent: x={goal_cfg["x"]}, y={goal_cfg["y"]}, yaw={yaw:.3f} rad'
        )

    # =========================================================================
    # Step 4 — Spawn obstacle
    # =========================================================================

    def _spawn_obstacle(self):
        obs_cfg = self._config.get('obstacle_to_spawn')
        if not obs_cfg:
            self.get_logger().info('[Step 4] No obstacle configured — skipping.')
            return

        sdf_filename = obs_cfg.get('sdf_filename', '')
        if not sdf_filename:
            self.get_logger().error('[Step 4] "sdf_filename" missing from obstacle_to_spawn.')
            return

        sdf_path = os.path.join(
            self._pkg_share, 'testing', 'obstacles', sdf_filename
        )
        if not os.path.exists(sdf_path):
            self.get_logger().error(
                f'[Step 4] SDF not found: {sdf_path}\n'
                f'  Run "colcon build" to install the testing/ directory.'
            )
            return

        pose = obs_cfg.get('pose', {})
        obstacle_name = obs_cfg.get('name', 'test_obstacle')

        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'default',
            '-name',  obstacle_name,
            '-file',  sdf_path,
            '-x', str(pose.get('x',     0.0)),
            '-y', str(pose.get('y',     0.0)),
            '-z', str(pose.get('z',     0.25)),
            '-R', str(pose.get('roll',  0.0)),
            '-P', str(pose.get('pitch', 0.0)),
            '-Y', str(pose.get('yaw',   0.0)),
        ]

        self.get_logger().info(f'[Step 4] Spawning obstacle "{obstacle_name}"...')

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=15.0)
            if result.returncode == 0:
                self.get_logger().info(
                    f'[Step 4] Obstacle "{obstacle_name}" spawned at '
                    f'x={pose.get("x", 0.0)}, y={pose.get("y", 0.0)}.'
                )
            else:
                self.get_logger().error(
                    f'[Step 4] Spawn failed (rc={result.returncode}).\n'
                    f'  stdout: {result.stdout.strip()}\n'
                    f'  stderr: {result.stderr.strip()}'
                )
        except subprocess.TimeoutExpired:
            self.get_logger().error('[Step 4] Obstacle spawn timed out after 15s.')
        except FileNotFoundError:
            self.get_logger().error(
                '[Step 4] "ros2" executable not found. '
                'Make sure the ROS2 environment is sourced.'
            )


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = TestRunnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
