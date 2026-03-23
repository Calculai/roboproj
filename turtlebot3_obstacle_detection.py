#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan


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

        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

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

        self.cmd_vel_pub.publish(twist)


def main(args=None):

    rclpy.init(args=args)

    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()

    rclpy.spin(turtlebot3_obstacle_detection)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()