from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
import sys
import time
import numpy as np
from tf2_ros import Buffer, TransformListener
from collections import deque

if os.name == 'nt':
    import msvcrt
else:
    import select
    import termios
    import tty


class Turtlebot3ObstacleMapping(Node):

    def __init__(self):
        # ROS2 node that combines:
        # 1) reactive maze exploration motion
        # 2) occupancy-grid map building
        # 3) runtime logging + keyboard/timeout shutdown
        super().__init__('turtlebot3_obstacle_mapping')

        print('TurtleBot3 Obstacle Mapping - Reactive Navigation + Occupancy Grid')
        print('------------------------------------------------------------------')
        print('Escapepath method + map building from LaserScan')
        print('stop distance: 0.22 m')
        print('------------------------------------------------------------------')

        # Latest lidar message is cached here and reused by navigation + mapping.
        self.scan_msg = None
        self.has_scan_received = False

        # Navigation tuning parameters.
        self.stop_distance = 0.22
        self.clearance_distance = 0.30
        self.wall_follow_distance = 0.32
        self.turn_speed = 0.8
        self.follow_turn_speed = 0.35
        self.min_linear_speed = 0.08
        self.max_linear_speed = 0.32

        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.2
        self.tele_twist.angular.z = 0.0

        # TF listener is used to get robot pose in map frame.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.visited = set()

        qos_cmd = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_cmd)

        qos_map = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', qos_map)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos_profile=qos_profile_sensor_data
        )

        # Runtime stats and behavior state tracking.
        self.speed_updates = 0
        self.speed_accumulation = 0.0
        self.last_mode = None
        self.recent_cells = deque(maxlen=30)

        self.max_runtime_seconds = 120.0
        self.start_time = time.monotonic()

        self.shutdown_key = 'q'
        self.keyboard_available = False
        self.shutdown_requested = False
        self.stdin_fd = None
        self.original_terminal_settings = None
        self.setup_keyboard_shutdown()

        # Occupancy grid settings (-1 unknown, 0 free, 100 occupied).
        self.map_resolution = 0.05
        self.map_width = 240
        self.map_height = 240
        self.map_origin_x = -(self.map_width * self.map_resolution) / 2.0
        self.map_origin_y = -(self.map_height * self.map_resolution) / 2.0
        self.map_data = np.full((self.map_height, self.map_width), -1, dtype=np.int8)

        # Main update loop, periodic stats logging, keyboard polling, and map publish.
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.stats_timer = self.create_timer(5.0, self.log_speed_stats)
        self.shutdown_timer = self.create_timer(0.5, self.check_shutdown_key)
        self.map_timer = self.create_timer(1.0, self.publish_map)

    def scan_callback(self, msg):
        self.scan_msg = msg
        self.has_scan_received = True

    def get_robot_pose(self):
        # Returns current robot pose in map frame as (x, y, yaw).
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y

            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)

            return x, y, theta

        except Exception:
            return None

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

    def setup_keyboard_shutdown(self):
        # Non-blocking keyboard setup so 'q' can stop the node gracefully.
        try:
            if os.name == 'nt':
                self.keyboard_available = True
            elif sys.stdin.isatty():
                self.stdin_fd = sys.stdin.fileno()
                self.original_terminal_settings = termios.tcgetattr(self.stdin_fd)
                tty.setcbreak(self.stdin_fd)
                self.keyboard_available = True
        except Exception as error:
            self.get_logger().warn(f'Keyboard shutdown setup failed: {error}')

        if self.keyboard_available:
            self.get_logger().info(f"Press '{self.shutdown_key}' to shutdown.")
        else:
            self.get_logger().warn('Keyboard shutdown unavailable in this terminal.')

        self.get_logger().info(f'Auto shutdown after {self.max_runtime_seconds:.0f} seconds.')

    def check_shutdown_key(self):
        if not self.keyboard_available:
            return

        key = None

        if os.name == 'nt':
            if msvcrt.kbhit():
                key = msvcrt.getwch()
        else:
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
                key = sys.stdin.read(1)

        if key and key.lower() == self.shutdown_key:
            self.get_logger().info('Shutdown key pressed. Exiting node...')
            self.shutdown_requested = True

    def log_speed_stats(self):
        if self.speed_updates > 0:
            elapsed_seconds = time.monotonic() - self.start_time
            average_speed = self.speed_accumulation / self.speed_updates
            self.get_logger().info(
                f'Elapsed Time: {elapsed_seconds:.1f}s, '
                f'Speed Updates: {self.speed_updates}, '
                f'Speed Accumulation: {self.speed_accumulation:.2f}, '
                f'Average Linear Speed: {average_speed:.4f} m/s'
            )

    def timer_callback(self):
        # Run navigation+mapping step only after at least one scan is received.
        if self.has_scan_received:
            self.detect_obstacle_and_map()

    def world_to_grid(self, x, y):
        # Convert world coordinates (meters) to occupancy-grid indices.
        gx = int((x - self.map_origin_x) / self.map_resolution)
        gy = int((y - self.map_origin_y) / self.map_resolution)

        if gx < 0 or gy < 0 or gx >= self.map_width or gy >= self.map_height:
            return None
        return gx, gy

    def bresenham(self, x0, y0, x1, y1):
        # Grid line rasterization used to mark free cells along each lidar beam.
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return points

    def update_map_from_scan(self, robot_x, robot_y, robot_theta):
        # For each valid lidar beam:
        # - mark intermediate cells as free
        # - mark endpoint as occupied (if obstacle hit)
        if self.scan_msg is None:
            return

        robot_cell = self.world_to_grid(robot_x, robot_y)
        if robot_cell is None:
            return

        rx, ry = robot_cell
        self.map_data[ry, rx] = 0

        ranges = self.scan_msg.ranges
        angle_min = self.scan_msg.angle_min
        angle_increment = self.scan_msg.angle_increment
        range_min = self.scan_msg.range_min
        range_max = self.scan_msg.range_max

        for index, distance in enumerate(ranges):
            if not np.isfinite(distance):
                continue
            if distance <= 0.0 or distance < range_min:
                continue

            capped_distance = min(distance, range_max)
            beam_angle = robot_theta + angle_min + index * angle_increment

            end_x = robot_x + capped_distance * math.cos(beam_angle)
            end_y = robot_y + capped_distance * math.sin(beam_angle)
            end_cell = self.world_to_grid(end_x, end_y)

            if end_cell is None:
                continue

            ex, ey = end_cell
            line_cells = self.bresenham(rx, ry, ex, ey)

            for free_x, free_y in line_cells[:-1]:
                if self.map_data[free_y, free_x] != 100:
                    self.map_data[free_y, free_x] = 0

            if distance < (range_max * 0.995):
                self.map_data[ey, ex] = 100

    def publish_map(self):
        # Publish current occupancy grid for visualization/consumers.
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.orientation.w = 1.0

        msg.data = self.map_data.flatten().tolist()
        self.map_pub.publish(msg)

    def _normalize_angle(self, angle):
        # Keep angles in [-pi, pi] for robust angle comparisons.
        return math.atan2(math.sin(angle), math.cos(angle))

    def _window_min_distance(self, center_angle, half_width):
        # Returns minimum lidar distance inside an angular window.
        if self.scan_msg is None or not self.scan_msg.ranges:
            return float('inf')

        min_distance = float('inf')
        angle = self.scan_msg.angle_min

        for distance in self.scan_msg.ranges:
            if np.isfinite(distance) and distance > 0.0:
                delta = self._normalize_angle(angle - center_angle)
                if abs(delta) <= half_width and distance < min_distance:
                    min_distance = distance
            angle += self.scan_msg.angle_increment

        return min_distance

    def _publish_mode_change(self, mode):
        if mode != self.last_mode:
            self.get_logger().info(f'Explore mode: {mode}')
            self.last_mode = mode

    def _scan_speed_factor(self, front_distance):
        # Speed scaling based on immediate frontal clearance.
        # Closer obstacle => lower factor.
        if not np.isfinite(front_distance):
            return 0.55

        if front_distance <= self.stop_distance:
            return 0.0

        if front_distance <= 0.32:
            return 0.45

        if front_distance <= 0.5:
            return 0.65

        if front_distance <= 0.8:
            return 0.85

        return 1.0

    def _map_speed_factor(self, robot_x, robot_y, robot_theta):
        # Speed scaling based on known map quality ahead.
        # More known-free cells => faster; unknown/occupied ahead => slower.
        sample_distances = [0.20, 0.40, 0.60, 0.80, 1.00]
        free_score = 0.0
        occupancy_penalty = 0.0
        unknown_penalty = 0.0

        for distance in sample_distances:
            sample_x = robot_x + distance * math.cos(robot_theta)
            sample_y = robot_y + distance * math.sin(robot_theta)
            cell = self.world_to_grid(sample_x, sample_y)

            if cell is None:
                unknown_penalty += 0.35
                continue

            gx, gy = cell
            value = int(self.map_data[gy, gx])

            if value == 0:
                free_score += 1.0
            elif value == 100:
                occupancy_penalty += 1.2
            else:
                unknown_penalty += 0.5

        combined = free_score - occupancy_penalty - unknown_penalty

        if combined >= 3.0:
            return 1.0
        if combined >= 1.0:
            return 0.85
        if combined >= -0.5:
            return 0.65
        return 0.45

    def detect_obstacle_and_map(self):
        # Main control step:
        # 1) compute speed target from user input + scan/map scaling
        # 2) pick exploration action (turn/follow wall/forward)
        # 3) update map
        # 4) publish cmd_vel and stats
        user_speed = max(self.min_linear_speed, min(self.tele_twist.linear.x, self.max_linear_speed))
        if self.scan_msg is None or not self.scan_msg.ranges:
            self.get_logger().warn('No scan data available to detect obstacles.')
            return

        pose = self.get_robot_pose()
        if pose is None:
            self.get_logger().warn('Failed to get robot pose.')
            return

        x, y, theta = pose

        twist = Twist()

        front_distance = self._window_min_distance(center_angle=0.0, half_width=math.radians(20.0))
        right_distance = self._window_min_distance(center_angle=-math.pi / 2.0, half_width=math.radians(25.0))
        left_distance = self._window_min_distance(center_angle=math.pi / 2.0, half_width=math.radians(25.0))
        scan_factor = self._scan_speed_factor(front_distance)
        map_factor = self._map_speed_factor(x, y, theta)
        commanded_speed = max(self.min_linear_speed, min(user_speed * scan_factor * map_factor, self.max_linear_speed))

        # Right-hand wall-following exploration policy with safety overrides.
        if front_distance < self.stop_distance and left_distance < self.stop_distance and right_distance < self.stop_distance:
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
            self._publish_mode_change('dead_end_turn')
        elif front_distance < self.stop_distance:
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed if left_distance >= right_distance else -self.turn_speed
            self._publish_mode_change('front_blocked_turn')
        elif right_distance > (self.wall_follow_distance + 0.12) and front_distance > self.clearance_distance:
            twist.linear.x = commanded_speed * 0.75
            twist.angular.z = -self.follow_turn_speed
            self._publish_mode_change('search_right_wall')
        elif right_distance < (self.wall_follow_distance - 0.10):
            twist.linear.x = commanded_speed * 0.7
            twist.angular.z = self.follow_turn_speed
            self._publish_mode_change('too_close_right_wall')
        else:
            twist.linear.x = commanded_speed
            twist.angular.z = 0.0
            self._publish_mode_change('forward_follow_wall')

        key = (round(x, 1), round(y, 1))
        self.visited.add(key)
        self.recent_cells.append(key)

        # Loop-recovery: if we keep revisiting same small area, force a turn.
        if len(self.recent_cells) == self.recent_cells.maxlen:
            unique_recent = len(set(self.recent_cells))
            if unique_recent <= 6:
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed
                self._publish_mode_change('loop_recovery_turn')

        self.update_map_from_scan(x, y, theta)

        self.speed_updates += 1
        self.speed_accumulation += twist.linear.x

        self.cmd_vel_pub.publish(twist)

    def destroy_node(self):
        # Graceful shutdown:
        # restore terminal state (Linux), stop robot, publish final map and logs.
        if os.name != 'nt' and self.original_terminal_settings is not None and self.stdin_fd is not None:
            try:
                termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.original_terminal_settings)
            except Exception as error:
                self.get_logger().warn(f'Failed to restore terminal settings: {error}')

        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_twist)
        time.sleep(0.1)

        self.publish_map()
        self.log_speed_stats()

        super().destroy_node()


def main(args=None):
    # Standard ROS2 node lifecycle + manual spin loop for timeout/shutdown checks.
    rclpy.init(args=args)

    turtlebot3_obstacle_mapping = Turtlebot3ObstacleMapping()

    try:
        while rclpy.ok() and not turtlebot3_obstacle_mapping.shutdown_requested:
            elapsed_seconds = time.monotonic() - turtlebot3_obstacle_mapping.start_time
            if elapsed_seconds >= turtlebot3_obstacle_mapping.max_runtime_seconds:
                turtlebot3_obstacle_mapping.get_logger().info(
                    'Auto shutdown timeout reached (120 seconds). Exiting node...'
                )
                turtlebot3_obstacle_mapping.shutdown_requested = True
                continue

            rclpy.spin_once(turtlebot3_obstacle_mapping, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        turtlebot3_obstacle_mapping.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
