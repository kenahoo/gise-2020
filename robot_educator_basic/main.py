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


#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep

btn = Button()


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


# basketball_mover()

# if pressing down button:
step_counter_pull_up_bar_dance_battle()

# dance_mission()

def left(state):
    """
     "state" tells us whether the button was pressed, or was released
    """
    if state:
        basketball_mover()
    
def right(state):
    if state:
        step_counter_pull_up_bar_dance_battle()
    
# def up(state):
#     print('Up button pressed' if state else 'Up button released')
    
# def down(state):
#     print('Down button pressed' if state else 'Down button released')
    
# def enter(state):
#     print('Enter button pressed' if state else 'Enter button released')
    
# def backspace(state):
#     print('Backspace button pressed' if state else 'Backspace button released')
    
# If running this script via SSH, press Ctrl+C to quit
# if running this script from Brickman, long-press backspace button to quit


btn.on_left = left
btn.on_right = right
# btn.on_up = up
# btn.on_down = down
# btn.on_enter = enter
# btn.on_backspace = backspace

while True:  # This loop checks buttons state continuously, 
             # calls appropriate event handlers
    btn.process() # Check for currently pressed buttons. 
    # If the new state differs from the old state, 
    # call the appropriate button event handlers.
    sleep(0.01)  # buttons state will be checked every 0.01 second

