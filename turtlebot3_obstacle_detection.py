#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");

from geometry_msgs.msg import Twist
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import sys
import time

if os.name == 'nt':
    import msvcrt
else:
    import select
    import termios
    import tty


class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')

        print('TurtleBot3 Obstacle Detection - Quadrant Navigation')
        print('----------------------------------------------')
        print('Escapepath method')
        print('stop distance: 0.20 m')
        print('----------------------------------------------')

        self.scan_ranges = []
        self.has_scan_received = False

        self.stop_distance = 0.2

        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.2
        self.tele_twist.angular.z = 0.0

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

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

        # Speed tracking
        self.speed_updates = 0
        self.speed_accumulation = 0.0

        # Auto shutdown after fixed runtime
        self.max_runtime_seconds = 120.0
        self.start_time = time.monotonic()

        # Keyboard shutdown tracking
        self.shutdown_key = 'q'
        self.keyboard_available = False
        self.shutdown_requested = False
        self.stdin_fd = None
        self.original_terminal_settings = None
        self.setup_keyboard_shutdown()

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.stats_timer = self.create_timer(5.0, self.log_speed_stats)
        self.shutdown_timer = self.create_timer(0.5, self.check_shutdown_key)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

    def setup_keyboard_shutdown(self):
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
        if self.has_scan_received:
            self.detect_obstacle()

    def detect_obstacle(self):

        # angular velocity for turns
        A = 0.5

        # linear velocity for turns
        L = 0.0

        # Sector boundaries
        left_inner_limit = int(len(self.scan_ranges) / 8)        # 45°
        left_outer_limit = int(len(self.scan_ranges) / 4)        # 90°

        right_outer_limit = int(len(self.scan_ranges) * 3 / 4)   # 270°
        right_inner_limit = int(len(self.scan_ranges) * 7 / 8)   # 315°

        # Extract sectors and remove invalid values
        left_inner  = [r for r in self.scan_ranges[0:left_inner_limit] if 0.12 < r < 3.5]
        left_outer  = [r for r in self.scan_ranges[left_inner_limit:left_outer_limit] if 0.12 < r < 3.5]

        right_outer = [r for r in self.scan_ranges[right_outer_limit:right_inner_limit] if 0.12 < r < 3.5]
        right_inner = [r for r in self.scan_ranges[right_inner_limit:] if 0.12 < r < 3.5]

        # Compute minimum distances
        dist_left_outer = min(left_outer) if left_outer else float('inf')
        dist_left_inner = min(left_inner) if left_inner else float('inf')

        dist_right_inner = min(right_inner) if right_inner else float('inf')
        dist_right_outer = min(right_outer) if right_outer else float('inf')

        twist = Twist()

        # Decision logic using inner sectors first
        if dist_left_inner < self.stop_distance:

            if dist_right_inner > self.stop_distance:
                twist.linear.x = L 
                twist.angular.z = -A
                self.get_logger().info(f'Inner LEFT blocked ({dist_left_inner:.2f}m). Turning RIGHT.')

            elif dist_right_outer > self.stop_distance:
                twist.linear.x = L
                twist.angular.z = -A
                self.get_logger().info('Inner sectors blocked. Using outer RIGHT.')

            elif dist_left_outer > self.stop_distance:
                twist.linear.x = L
                twist.angular.z = A
                self.get_logger().info('Inner sectors blocked. Using outer LEFT.')

            else:
                twist.linear.x = 0.0
                twist.angular.z = A
                self.get_logger().info('Surrounded. Rotating LEFT.')

        elif dist_right_inner < self.stop_distance:

            if dist_left_inner > self.stop_distance:
                twist.linear.x = L
                twist.angular.z = A
                self.get_logger().info(f'Inner RIGHT blocked ({dist_right_inner:.2f}m). Turning LEFT.')

            elif dist_left_outer > self.stop_distance:
                twist.linear.x = L
                twist.angular.z = A
                self.get_logger().info('Inner sectors blocked. Using outer LEFT.')

            elif dist_right_outer > self.stop_distance:
                twist.linear.x = L
                twist.angular.z = -A
                self.get_logger().info('Inner sectors blocked. Using outer RIGHT.')

            else:
                twist.linear.x = 0.0
                twist.angular.z = -A
                self.get_logger().info('Surrounded. Rotating RIGHT.')

        else:
            twist = self.tele_twist

        # Track speed updates
        self.speed_updates += 1
        self.speed_accumulation += twist.linear.x

        self.cmd_vel_pub.publish(twist)

    def destroy_node(self):
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

        self.log_speed_stats()
        
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()

    try:
        while rclpy.ok() and not turtlebot3_obstacle_detection.shutdown_requested:
            elapsed_seconds = time.monotonic() - turtlebot3_obstacle_detection.start_time
            if elapsed_seconds >= turtlebot3_obstacle_detection.max_runtime_seconds:
                turtlebot3_obstacle_detection.get_logger().info(
                    'Auto shutdown timeout reached (120 seconds). Exiting node...'
                )
                turtlebot3_obstacle_detection.shutdown_requested = True
                continue

            rclpy.spin_once(turtlebot3_obstacle_detection, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        turtlebot3_obstacle_detection.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()