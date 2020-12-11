#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Driving Base Program
-----------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
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
    robot.straight(350)
    robot.straight(-350)

def step_counter():
    robot.straight(650)
    robot.stop()
    robot.settings(straight_speed=20, straight_acceleration=791, turn_rate=30, turn_acceleration=735)
    robot.straight(265)
    robot.stop()
    robot.settings(straight_speed=900, straight_acceleration=791, turn_rate=30, turn_acceleration=735)
    robot.straight(-1066)


robot = robot_setup()

# basketball_mover()

step_counter()
