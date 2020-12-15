#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Driving Base Program
-----------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot

PyBricks docs:  https://pybricks.github.io/ev3-micropython/
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop
from pybricks.robotics import DriveBase

import time


def robot_setup():
    # Initialize the EV3 Brick.
    ev3 = EV3Brick()

    # Initialize the motors.
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.B)

    # Initialize the drive base.
    robot = DriveBase(left_motor, right_motor, wheel_diameter=85.0, axle_track=123.3)

    # Go forward and backwards
    robot.settings(straight_speed=2000, straight_acceleration=791, turn_rate=30, turn_acceleration=735)

    return robot


def basketball_mover():
    # Setup - hey guys the position is: (x, y) = (9, 3)
    robot.straight(350)
    robot.straight(-350)

def step_counter():
    # Setup - close to the edge, line as far forward as possible
    # Only does step counter mission - we also have one that does step counter plus pull-up plus dancing.
    go_straight(650)
    go_straight(265, speed=20)
    go_straight(-1066, speed=900)


def dance_mission():
    # Setup - close to the edge, line as far forward as possible
    go_straight(650)
    go_straight(265, speed=20)
    
    # 1. go back a little
    go_straight(-250)
    # 2. turn left 90º
    robot.turn(-90)
    # 3. go straight for quite a while
    # 4. turn left 45º
    # 5. go forward
    # 6. dance & music

def go_straight(distance,speed=500):
    """
    docstring
    """
    robot.stop()
    robot.settings(straight_speed=speed, straight_acceleration=791, turn_rate=30, turn_acceleration=735)
    robot.straight(distance)

def turn(angle, speed=500, sharpness=30):
    robot.stop()
    robot.settings(straight_speed=speed, straight_acceleration=791, turn_rate=sharpness, turn_acceleration=735)
    robot.turn(angle)

robot = robot_setup()

robot.left.run_angle(speed=100, rotation_angle=100, then=Stop.HOLD, wait=False)
robot.right.run_angle(speed=-100, rotation_angle=100, then=Stop.HOLD, wait=True)

# if pressing up button:

# basketball_mover()

# if pressing down button:
# step_counter()

# dance_mission()
