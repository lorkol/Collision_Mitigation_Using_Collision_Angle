#!/usr/bin/env python3

# =============================================================================
# test_runner_node.py
#
# SEQUENCE:
#   1. Publish /initialpose so AMCL knows where the robot is
#      (must match the spawn pose in test_configuration.json)
#   2. Wait for bt_navigator lifecycle state == 'active'
#   3. Send the navigation goal — Nav2 takes over from here
#   4. Monitor robot velocity.  Once it exceeds spawn_velocity_threshold
#      (from obstacle_to_spawn config), spawn the obstacle at
#      spawn_distance_ahead metres in front of the robot.
#
# THREADING MODEL:
#   Main thread : rclpy.spin() — processes all callbacks.
#   Sequence thread (daemon): blocking startup sequence.
#     Uses time.sleep() for waiting, future.done() polling for service calls.
#     NEVER calls rclpy.spin*().
#   Spawn thread (daemon): created once inside _odom_callback when the
#     velocity threshold is crossed. Runs subprocess for gz entity creation.
#   All subscriptions and service clients are created in __init__.
#   threading.Event / threading.Lock are used for cross-thread signalling.
# =============================================================================

import json
import math
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
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState
from tf_transformations import euler_from_quaternion, quaternion_from_euler
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
        self.declare_parameter('robot_mass', 1.0)
        self.declare_parameter('collision_velocity_threshold', 0.1)
        self.declare_parameter('max_decel', 0.1)

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

        # --- Sequence synchronisation ---
        self._amcl_event = threading.Event()

        # --- Robot state (updated by _odom_callback) ---
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._last_velocity_x = 0.0

        # --- Obstacle spawning ---
        self._monitor_active = False
        self._obstacle_spawned = False
        self._spawn_lock = threading.Lock()

        # --- Collision detection ---
        self._collision_detected = False
        self._robot_mass = self.get_parameter('robot_mass').value
        self._collision_threshold = self.get_parameter('collision_velocity_threshold').value
        self._last_odom_stamp_ns = None
        # Deceleration spikes larger than 5× max_decel indicate physical contact, not MPPI braking.
        _COLLISION_ACCEL_MULTIPLIER = 5.0
        _max_decel = abs(self.get_parameter('max_decel').value)
        self._collision_accel_threshold = -_max_decel * _COLLISION_ACCEL_MULTIPLIER

        obs_cfg = self._config.get('obstacle_to_spawn', {})
        self.get_logger().info(
            f'Obstacle spawn trigger: velocity >= '
            f'{obs_cfg.get("spawn_velocity_threshold", 0.1):.2f} m/s, '
            f'distance ahead = {obs_cfg.get("spawn_distance_ahead", 1.5):.2f} m'
        )

        # Publishers
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', _QOS_DEFAULT
        )
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', _QOS_DEFAULT)

        # Subscriptions
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_callback,
            _QOS_TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Odometry,
            '/odom_ground_truth',
            self._odom_callback,
            _QOS_DEFAULT,
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
        self._publish_initial_pose()

        # Step 2: Wait until the full Nav2 navigation stack is active.
        self._wait_for_nav2_active()

        # Step 3: Send the navigation goal.
        self._send_goal()

        # Step 4: Activate monitoring — obstacle spawns automatically via
        # _odom_callback once the robot's velocity crosses the threshold.
        self.get_logger().info(
            '=== Goal sent. Obstacle will spawn once velocity threshold is crossed. ==='
        )
        self._monitor_active = True

    # =========================================================================
    # Step 1 — Set AMCL initial pose
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
        msg.pose.covariance[0]  = 0.25
        msg.pose.covariance[7]  = 0.02
        msg.pose.covariance[35] = 0.06

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
    # Step 2 — Wait for bt_navigator active
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
    # Odometry callback — robot pose tracking, collision detection,
    # and velocity-triggered obstacle spawning
    # =========================================================================

    def _odom_callback(self, msg: Odometry):
        # Always track pose so the spawn position is current.
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self._robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if not self._monitor_active:
            return

        current_vel = msg.twist.twist.linear.x

        # --- Velocity-triggered obstacle spawn ---
        obs_cfg = self._config.get('obstacle_to_spawn', {})
        vel_threshold = float(obs_cfg.get('spawn_velocity_threshold', 0.1))
        distance_ahead = float(obs_cfg.get('spawn_distance_ahead', 1.5))

        with self._spawn_lock:
            if not self._obstacle_spawned and current_vel >= vel_threshold:
                self._obstacle_spawned = True
                spawn_x = self._robot_x + distance_ahead * math.cos(self._robot_yaw)
                spawn_y = self._robot_y + distance_ahead * math.sin(self._robot_yaw)
                self.get_logger().info(
                    f'[Spawn trigger] velocity={current_vel:.3f} m/s >= threshold '
                    f'{vel_threshold:.3f} m/s — spawning obstacle at '
                    f'x={spawn_x:.2f}, y={spawn_y:.2f}'
                )
                t = threading.Thread(
                    target=self._spawn_obstacle,
                    args=(spawn_x, spawn_y),
                    daemon=True,
                )
                t.start()

        # --- Collision detection ---
        # Detect physical contact via a sudden deceleration spike.  With smoother
        # max_decel = 0.1 m/s², normal MPPI braking produces at most 0.005 m/s drop
        # per 50 ms odom step.  Any deceleration beyond collision_accel_threshold
        # (5 × max_decel = 0.5 m/s²) is a physical contact event, regardless of
        # whether the robot fully stops (angled collisions often do not stop the robot).
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if not self._collision_detected and self._obstacle_spawned and self._last_odom_stamp_ns is not None:
            dt = (stamp_ns - self._last_odom_stamp_ns) * 1e-9
            if dt > 0:
                apparent_accel = (current_vel - self._last_velocity_x) / dt
                if (self._last_velocity_x > self._collision_threshold
                        and apparent_accel < self._collision_accel_threshold):
                    self._collision_detected = True
                    self._monitor_active = False

                    delta_v = current_vel - self._last_velocity_x
                    delta_p = self._robot_mass * delta_v

                    self.get_logger().info('=' * 40)
                    self.get_logger().info('COLLISION DETECTED!')
                    self.get_logger().info(f'  Velocity Before: {self._last_velocity_x:.4f} m/s')
                    self.get_logger().info(f'  Velocity After:  {current_vel:.4f} m/s')
                    self.get_logger().info(f'  Apparent decel:  {apparent_accel:.3f} m/s²  '
                                           f'(threshold: {self._collision_accel_threshold:.3f} m/s²)')
                    self.get_logger().info(f'  Robot Mass:      {self._robot_mass:.2f} kg')
                    self.get_logger().info(f'  Change in Momentum (Δp): {delta_p:.4f} kg*m/s')
                    self.get_logger().info('=' * 40)

        self._last_odom_stamp_ns = stamp_ns
        self._last_velocity_x = current_vel

    # =========================================================================
    # Spawn obstacle at a computed world position
    # =========================================================================

    def _spawn_obstacle(self, spawn_x: float, spawn_y: float):
        obs_cfg = self._config.get('obstacle_to_spawn')
        if not obs_cfg:
            self.get_logger().info('[Spawn] No obstacle configured — skipping.')
            return

        sdf_filename = obs_cfg.get('sdf_filename', '')
        if not sdf_filename:
            self.get_logger().error('[Spawn] "sdf_filename" missing from obstacle_to_spawn.')
            return

        sdf_path = os.path.join(
            self._pkg_share, 'testing', 'obstacles', sdf_filename
        )
        if not os.path.exists(sdf_path):
            self.get_logger().error(
                f'[Spawn] SDF not found: {sdf_path}\n'
                f'  Run "colcon build" to install the testing/ directory.'
            )
            return

        obstacle_name = obs_cfg.get('name', 'test_obstacle')
        z     = float(obs_cfg.get('z',     0.25))
        roll  = float(obs_cfg.get('roll',  0.0))
        pitch = float(obs_cfg.get('pitch', 0.0))
        yaw   = float(obs_cfg.get('yaw',   0.0))

        q = quaternion_from_euler(roll, pitch, yaw)

        # Use `gz service` directly instead of `ros2 run ros_gz_sim create`.
        # gz is a compiled C++ binary with no ROS2 init overhead, reducing
        # spawn latency from ~0.5–2 s to ~50–100 ms for consistent collision timing.
        req = (
            f'sdf_filename: "{sdf_path}" '
            f'name: "{obstacle_name}" '
            f'pose {{ '
            f'position {{ x: {spawn_x} y: {spawn_y} z: {z} }} '
            f'orientation {{ x: {q[0]} y: {q[1]} z: {q[2]} w: {q[3]} }} '
            f'}}'
        )
        cmd = [
            'gz', 'service',
            '-s', '/world/default/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', req,
        ]

        self.get_logger().info(f'[Spawn] Spawning "{obstacle_name}" at x={spawn_x:.2f}, y={spawn_y:.2f}...')

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10.0)
            if result.returncode == 0:
                self.get_logger().info(f'[Spawn] "{obstacle_name}" spawned successfully.')
            else:
                self.get_logger().error(
                    f'[Spawn] Spawn failed (rc={result.returncode}).\n'
                    f'  stdout: {result.stdout.strip()}\n'
                    f'  stderr: {result.stderr.strip()}'
                )
        except subprocess.TimeoutExpired:
            self.get_logger().error('[Spawn] Obstacle spawn timed out after 10s.')
        except FileNotFoundError:
            self.get_logger().error(
                '[Spawn] "gz" executable not found. '
                'Make sure the Gazebo environment is sourced.'
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
