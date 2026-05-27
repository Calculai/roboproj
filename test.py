#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");

# maybe add the mapping idea (make a dataset of all points and figure out valid spots and then make the minimum valid spot)
# maybe make it find spots and then develop a curve (but issue is how to express it with twist when it is unreliable)
# somehow make the map disfavor the direction it has already been if it encounters later

from geometry_msgs.msg import Twist
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import time
import numpy as np
from smbus2 import SMBus
import RPi.GPIO as GPIO
from gpiozero import LED

import sys

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

        # initialize variables for laser scan data
        self.scan_ranges = []
        self.has_scan_received = False

        # RGB sensor setup
        self.i2c_bus = None
        self.light_sensor_address = 0x44
        self.latest_rgb = {'red': 0, 'green': 0, 'blue': 0}
        self.light_sensor_enabled = self.setup_light_sensor()

        # LED setup for red detection feedback
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.led = LED(23)
        self.blinking = False
        self.blink_start = 0.0
        self.last_toggle = 0.0
        self.led_state = False
        self.last_blink_trigger = 0.0
        self.blink_cooldown = 1.0
        self.targets_found = 0

        # obstacle avoidance parameters
        self.stop_distance = 0.20
        self.max_linear_velocity = 0.22

        # teleoperation command storage for when no obstacles are detected
        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.2
        self.tele_twist.angular.z = 0.0

        # publisher and subscriber setup
        qos = QoSProfile(depth=10)

        # publish velocity commands to cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # subscribe to laser scan data from the 'scan' topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )

        # subscribe to teleoperation commands from the 'cmd_vel_raw' topic
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos_profile=qos_profile_sensor_data
        )

        # speed tracking setup
        self.speed_updates = 0
        self.speed_accumulation = 0.0

        # auto shutdown setup
        self.max_runtime_seconds = 120.0
        self.start_time = time.monotonic()

        # keyboard shutdown setup
        self.shutdown_key = 'q'
        self.keyboard_available = False
        self.shutdown_requested = False
        self.stdin_fd = None
        self.original_terminal_settings = None
        self.setup_keyboard_shutdown()

        # collision counter setup
        self.collision_threshold = 0.158  # Distance to count a collision
        self.collision_count = 0
        self.collision_cooldown = 2.0     # Seconds between allowed counts
        self.last_collision_time = 0.0

        # set up ROS schedule for every periodic task (time intervals chosen to balance responsiveness and CPU usage)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.stats_timer = self.create_timer(5.0, self.log_speed_stats)
        self.shutdown_timer = self.create_timer(0.5, self.check_shutdown_key)
        self.colour_timer = self.create_timer(0.2, self.update_colour_sensor)
        self.blink_timer = self.create_timer(0.05, self.update_blink)

    def setup_light_sensor(self):
        try:
            self.i2c_bus = SMBus(1)
            self.i2c_bus.write_byte_data(self.light_sensor_address, 0x01, 0x05)
            time.sleep(0.5)
            self.get_logger().info('ISL29125 colour sensor initialized.')
            return True
        except Exception as error:
            self.get_logger().warn(f'Colour sensor setup failed: {error}')
            self.i2c_bus = None
            return False

    def trigger_blink(self):
        now = time.time()
        if now - self.last_blink_trigger >= self.blink_cooldown:
            self.blinking = True
            self.blink_start = now
            self.last_toggle = now
            self.last_blink_trigger = now
            self.targets_found += 1
            self.get_logger().info(f'Red dominance detected — LED blinking. Targets found: {self.targets_found}')

    def update_blink(self):
        if not self.blinking:
            return

        now = time.time()

        if now - self.blink_start >= 2.0:
            self.blinking = False
            self.led.off()
            self.led_state = False
            return

        if now - self.last_toggle >= 0.25:
            self.led_state = not self.led_state
            self.led.on() if self.led_state else self.led.off()
            self.last_toggle = now

    def update_colour_sensor(self):
        if not self.light_sensor_enabled or self.i2c_bus is None:
            return

        try:
            data = self.i2c_bus.read_i2c_block_data(self.light_sensor_address, 0x09, 6)

            green = (data[1] << 8) | data[0]
            red = (data[3] << 8) | data[2]
            blue = (data[5] << 8) | data[4]

            red = int(red * 1.3)/1000
            green = int(green * 0.75)/1000
            blue = int(blue * 1.25)/1000

            self.latest_rgb = {'red': red, 'green': green, 'blue': blue}

            # comment out in final version, useful for debugging
            #self.get_logger().info(
            #    f"RGB Values -> Red: {red} | Green: {green} | Blue: {blue}"
            #)

            # Trigger blink if red is significantly higher than both green and blue
            if red > green * 1.8 and red > blue * 1.8 and red > 0.05:
                self.trigger_blink()

        except Exception as error:
            self.get_logger().warn(f'Failed reading colour sensor: {error}')

    def update_collision_counter(self, min_inner_dist):
        current_time = time.time()

        is_colliding = min_inner_dist < self.collision_threshold
        cooldown_elapsed = (current_time - self.last_collision_time) > self.collision_cooldown

        if is_colliding and cooldown_elapsed:
            self.collision_count += 1
            self.last_collision_time = current_time
            self.get_logger().warn(
                f'COLLISION DETECTED! Dist: {min_inner_dist:.3f}m | Total: {self.collision_count}'
            )

    def scan_callback(self, msg):
        # store the latest scan ranges and set the flag to indicate we have received scan data
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def calculate_regression_speeds(self, distance):
        # base case for no obstacle or very close obstacle
        if distance >= 0.40:
            return 0.22, 0.0
        if distance <= 0.20:
            return 0.0, 1.8

        # L: linear velocity regression (1.1 * distance)
        v_linear = (0.55 * distance) 
        
        # A: angular velocity regression (-9.0 * distance + 3.6)
        v_angular = (-2.5 * distance) + 1
        
        return round(v_linear, 3), round(v_angular, 3)

    def cmd_vel_raw_callback(self, msg):
        # store the latest teleoperation command to use when no obstacles are detected
        self.tele_twist = msg

    def clamp_linear_velocity(self, linear_velocity):
        # ensure the linear velocity is within the defined limits
        return max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_velocity))

    def setup_keyboard_shutdown(self):
        # check if keyboard input is available
        try:
            if os.name == 'nt':
                self.keyboard_available = True
            elif sys.stdin.isatty():
                self.stdin_fd = sys.stdin.fileno()
                self.original_terminal_settings = termios.tcgetattr(self.stdin_fd)
                tty.setcbreak(self.stdin_fd)
                self.keyboard_available = True
        except Exception as error:
            # if not available, log a warning but continue without keyboard shutdown
            self.get_logger().warn(f'Keyboard shutdown setup failed: {error}')

        if self.keyboard_available:
            self.get_logger().info(f"Press '{self.shutdown_key}' to shutdown.")
        else:
            self.get_logger().warn('Keyboard shutdown unavailable in this terminal.')

        self.get_logger().info(f'Auto shutdown after {self.max_runtime_seconds:.0f} seconds.')

    def check_shutdown_key(self):
        # if keyboard shutdown is not available, skip checking
        if not self.keyboard_available:
            return

        # check for key press without blocking
        key = None
        if os.name == 'nt':
            if msvcrt.kbhit():
                key = msvcrt.getwch()
        else:
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
                key = sys.stdin.read(1)

        # if the shutdown key is pressed, set the shutdown_requested flag to True
        if key and key.lower() == self.shutdown_key:
            self.get_logger().info('Shutdown key pressed. Exiting node...')
            self.shutdown_requested = True

    def log_speed_stats(self):
        if self.speed_updates > 0:
            # calculate elapsed time and average speed, then log the stats
            elapsed_seconds = time.monotonic() - self.start_time
            average_speed = self.speed_accumulation / self.speed_updates
            self.get_logger().info(
                f'Elapsed Time: {elapsed_seconds:.1f}s, '
                f'Speed Updates: {self.speed_updates}, '
                f'Average Linear Speed: {average_speed:.4f} m/s, '
                f'Targets Found: {self.targets_found}, '
                f'Collisions: {self.collision_count}'
            )

    def timer_callback(self):
        # only attempt obstacle detection if we have received scan data
        if self.has_scan_received:
            self.detect_obstacle()

    def detect_obstacle(self):

        # set sector boundaries
        left_inner_limit = int(len(self.scan_ranges) / 8)        # 45°
        left_outer_limit = int(len(self.scan_ranges) / 4)        # 90°

        right_outer_limit = int(len(self.scan_ranges) * 3 / 4)   # 270°
        right_inner_limit = int(len(self.scan_ranges) * 7 / 8)   # 315°

        # extract sectors and remove invalid values
        left_inner  = [r for r in self.scan_ranges[0:left_inner_limit] if 0.12 < r < 3.5]
        left_outer  = [r for r in self.scan_ranges[left_inner_limit:left_outer_limit] if 0.12 < r < 3.5]

        right_outer = [r for r in self.scan_ranges[right_outer_limit:right_inner_limit] if 0.12 < r < 3.5]
        right_inner = [r for r in self.scan_ranges[right_inner_limit:] if 0.12 < r < 3.5]

        # extract minimum distances from each sector, defaulting to infinity if no valid readings
        dist_left_outer = min(left_outer) if left_outer else float('inf')
        dist_left_inner = min(left_inner) if left_inner else float('inf')

        dist_right_inner = min(right_inner) if right_inner else float('inf')
        dist_right_outer = min(right_outer) if right_outer else float('inf')

        # find minimum distance of inner two sectors
        x = min(dist_left_inner, dist_right_inner)

        # determine angular and linear velocity
        L, A = self.calculate_regression_speeds(x)
        # make sure the linear velocity is within limits
        L = self.clamp_linear_velocity(L)

        # collision counter 
        self.update_collision_counter(x)

        twist = Twist()
        
        #obstacle avoidance logic
        if x < 0.40:
            # use speed from regression
            twist.linear.x = L
            
            # curve in most open direction
            if dist_left_inner < dist_right_inner:
                twist.angular.z = -A # curve right
            else:
                twist.angular.z = A  # curve left
                
            # if closer than 0.20 we try to select an escape path and turn towards it
            if x < self.stop_distance:
                if dist_left_inner < self.stop_distance:
                    if dist_right_inner > self.stop_distance:
                        twist.angular.z = -A
                    elif dist_right_outer > self.stop_distance:
                        twist.angular.z = -A
                    elif dist_left_outer > self.stop_distance:
                        twist.angular.z = A
                elif dist_right_inner < self.stop_distance:
                    if dist_left_inner > self.stop_distance:
                        twist.angular.z = A
                    elif dist_left_outer > self.stop_distance:
                        twist.angular.z = A
                    elif dist_right_outer > self.stop_distance:
                        twist.angular.z = -A
        else:
            # if no close obstacles max speed ahead
            twist = self.tele_twist

        # track speed stats
        self.speed_updates += 1
        self.speed_accumulation += twist.linear.x

        # publish the velocity command
        self.cmd_vel_pub.publish(twist)

    def destroy_node(self):
        # if the I2C bus was initialized close it safely
        if self.i2c_bus is not None:
            try:
                self.i2c_bus.close()
            except Exception as error:
                # if closing fails, log a warning but continue with cleanup
                self.get_logger().warn(f'Failed to close I2C bus: {error}')

        # ensure the LED is turned off and GPIO is cleaned up safely
        try:
            self.led.off()
            GPIO.cleanup()
        except Exception as error:
            # if GPIO cleanup fails, log a warning but continue with shutdown
            self.get_logger().warn(f'Failed to clean up GPIO: {error}')

        # restore terminal settings if modified
        if os.name != 'nt' and self.original_terminal_settings is not None and self.stdin_fd is not None:
            try:
                termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.original_terminal_settings)
            except Exception as error:
                self.get_logger().warn(f'Failed to restore terminal settings: {error}')

        # publish a final zero velocity command to ensure the robot stops
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_twist)
        # wait briefly to ensure the stop command is sent before shutting down
        time.sleep(0.1)

        # log final speed stats before shutdown
        self.log_speed_stats()
        
        # call the superclass destroy_node to complete shutdown
        super().destroy_node()


def main(args=None):

    # initialize ROS 2 and create the node
    rclpy.init(args=args)

    # create the Turtlebot3ObstacleDetection node, thereby initializing all subscriptions, publishers, and timers
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    
    # main loop: spin the node until shutdown is requested (either by keyboard or auto timeout)
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
