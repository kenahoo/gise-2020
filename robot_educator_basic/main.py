#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Driving Base Program
-----------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot

PyBricks docs:  https://pybricks.github.io/ev3-micropython/

We are using some button techniques from https://pybricks.github.io/ev3-micropython/examples/elephant.html .
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop, Button
from pybricks.robotics import DriveBase

# from pybricks.parameters import Port, Direction, Color
from pybricks.tools import wait, StopWatch


import time


def robot_setup():
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

def go(distance, speed=400, turn_angle=0):
    """
    :param distance: how far to go, in millimeters
    :param speed: how fast to go, in unknown units, in the range -1800 to +1800
    :param turn_angle:
      * 0 for straight;              (left, right) = (  speed,   speed)
      * 50 for one-wheel turn right; (left, right) = (2*speed,       0)
      * 100 for turn in place right; (left, right) = (  speed,  -speed)
      * -50 for one-wheel turn left; (left, right) = (      0, 2*speed)
      * -100 for turn in place left; (left, right) = ( -speed,   speed)
    """
    # First: get ratio of left & right speeds
    # Then: multiply by 'speed' number
    left_motor_speed = speed/100 * turn_angle + 0.5*speed   # 100/100 * -100 + 0.5 * 100 = 1 * -100 + 50 = -50
    right_motor_speed = -speed/100 * turn_angle + 0.5*speed

    # when we asked for 20cm, it went 14cm.  So multiply by 10/7.
    distance_factor = 10/7

    avg_speed = (left_motor_speed + right_motor_speed) / 2
    print("left: " + str(left_motor_speed) + "; right: " + str(right_motor_speed))
    robot.left.run_angle(speed = left_motor_speed, rotation_angle=distance*left_motor_speed/avg_speed*distance_factor, then=Stop.HOLD, wait=False)
    robot.right.run_angle(speed = right_motor_speed, rotation_angle=distance*right_motor_speed/avg_speed*distance_factor, then=Stop.HOLD, wait=True)

robot = robot_setup()

# This is testing driving straight:
# go(400, speed=800)
def dance():
    go(20)
    go(-20)
    dance()

# This here is going to be go-under-pullup-bar-to-dance-area
def step_counter_pull_up_bar_dance_battle():
    go_straight(650)  # Forward
    go_straight(265, speed=20)  # Forward slowly
    robot.stop()
    go(distance=-350, speed=450, turn_angle=-5) # Backwards & to the left
    go(360.69, 440, -19) # Forwards and more sharply to the left
    go(distance=400, speed=450)  # Straight
    go(distance=50, turn_angle=-60)  # Turn left
    go(330, 450, 0)  # Go to dance area
    dance()

def line_follower(distance,speed,turn_angle):
    pass

def button_loop():
    # Initialize the EV3 Brick.
    ev3 = EV3Brick()

    while True:
        # Wait until any Brick Button is pressed.
        while not any(ev3.buttons.pressed()):
            wait(10)

        # Respond to the Brick Button press.

        # Check whether Up Button is pressed
        if Button.UP in ev3.buttons.pressed():
            ev3.speaker.beep(600)
            step_counter_pull_up_bar_dance_battle()

            # To avoid registering the same command again, wait until
            # the Up Button is released before continuing.
            while Button.UP in ev3.buttons.pressed():
                wait(10)

        # Check whether Down Button is pressed
        if Button.DOWN in ev3.buttons.pressed():
            ev3.speaker.beep(600)
            basketball_mover()

            # To avoid registering the same command again, wait until
            # the Down Button is released before continuing.
            while Button.DOWN in ev3.buttons.pressed():
                wait(10)

button_loop()
