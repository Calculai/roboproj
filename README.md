# TurtleBot3 Obstacle Detection and RGB Feedback

A first-semester computational technology project exploring TurtleBot3, Raspberry Pi, and Python. The project contains a ROS 2 node that uses laser scan data to avoid nearby obstacles and an ISL29125 RGB colour sensor to trigger an LED when it detects red-dominant light.

## What it does

The node in [`turtlebot3_obstacle_detection_rgb.py`](turtlebot3_obstacle_detection_rgb.py):

- Subscribes to laser scans on `scan`.
- Reads teleoperation velocity commands from `cmd_vel_raw`.
- Publishes the resulting velocity commands to `cmd_vel`.
- Slows and steers around nearby obstacles using distances from the left and right scan sectors.
- Stops forward motion when an obstacle is very close and attempts to turn toward a clearer direction.
- Checks the ISL29125 colour sensor over I²C; red-dominant readings trigger a GPIO LED blink and increment the detected-target count.
- Logs average commanded linear speed, elapsed time, target detections, and close-obstacle events.
- Shuts down after 120 seconds or when `q` is pressed in a compatible interactive terminal.

The colour sensor is optional at runtime: if setup or a read fails, the node logs a warning and continues without colour detection. Laser scan data is required for obstacle-avoidance decisions.

## Requirements

- A ROS 2 environment with `rclpy`, `geometry_msgs`, and `sensor_msgs` available
- A TurtleBot3 (or compatible robot) publishing `LaserScan` messages on `scan` and accepting `Twist` commands on `cmd_vel`
- A teleoperation source publishing `Twist` commands on `cmd_vel_raw` when teleoperation input is needed
- Raspberry Pi hardware with I²C enabled and an ISL29125 RGB sensor at address `0x44` for colour detection
- An LED connected to BCM GPIO pin 23 for visual feedback
- Python dependencies: `numpy`, `smbus2`, `RPi.GPIO`, and `gpiozero`

Use a ROS 2 distribution and TurtleBot3 setup compatible with the robot. This repository does not include a ROS package manifest or dependency installer.

## Run

In a terminal where the ROS 2 environment and robot topics are available:

```sh
python3 turtlebot3_obstacle_detection_rgb.py
```

The node publishes to `cmd_vel`; make sure no other controller is simultaneously commanding the same robot. Press `q` in the interactive terminal to request shutdown, or wait for the 120-second automatic shutdown. ROS shutdown and `Ctrl+C` also trigger cleanup.

## Default motion and detection settings

- Maximum commanded linear speed: `0.22 m/s`
- Default teleoperation command when no close obstacle is detected: `0.20 m/s` forward
- Obstacle avoidance activates below `0.40 m`
- Forward speed is set to zero at or below `0.20 m`, while the node attempts to turn toward an escape direction
- Collision events are logged below `0.158 m`, with a 2-second reporting cooldown
- Red-dominance detection triggers when the scaled red reading is more than 1.8 times both green and blue and exceeds `0.05`; the LED blinks for 2 seconds

These are values in the current script, not configurable ROS parameters.

## Safety

Test in a clear, controlled area at low speed with a person able to stop the robot immediately. Keep the robot's physical emergency stop accessible. This node provides basic reactive obstacle avoidance and should not be treated as a safety-rated controller; sensor gaps, topic or hardware failures, and obstacles outside the sensor's view can still lead to collisions.

## Repository contents

```text
.
├── README.md
└── turtlebot3_obstacle_detection_rgb.py
```
