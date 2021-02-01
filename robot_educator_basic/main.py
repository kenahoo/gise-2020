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
from pybricks.ev3devices import Motor, ColorSensor
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
    print(left_motor)
    print(right_motor)
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

def treadmill():
    # 1. Go forward a tiny bit
    go_straight(distance=70, speed=450)

    # 2. Line follow until you get to the treadmill
    line_follower(distance=1365, speed=100, gain=0.5, delay=1)

    # 3. Get up onto the treadmill
    # 4. Spin right wheel so we can spin the treadmill


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

        if Button.LEFT in ev3.buttons.pressed():
            ev3.speaker.beep(600)
            treadmill()

            while Button.LEFT in ev3.buttons.pressed():
                wait(10)



def line_follower(distance, speed, gain=1.2, right_side=True, delay=10):
    """
    Based on https://pybricks.github.io/ev3-micropython/examples/robot_educator_line.html

    :param gain: the gain of the proportional line controller. This means that for every
    percentage point of light deviating from the threshold, we set the turn
    rate of the drivebase to 1.2 degrees per second.
    """

    # This will measure the distance driven by using the "distance()" and "reset()" functions,
    # which are kind of like a Trip Odometer in a regular car.


    # Calculate the light threshold. Choose values based on your measurements.
    BLACK = 9
    WHITE = 85
    threshold = (BLACK + WHITE) / 2

    # Initialize the color sensor.
    line_sensor = ColorSensor(Port.S2)

    flip = -1 if right_side else 1

    robot.reset()
    gone_distance = 0

    # this loop goes until we've gone enough distance, then the condition will be false
    while gone_distance <= distance:
        # Calculate the deviation from the threshold.
        deviation = line_sensor.reflection() - threshold

        # Calculate the turn rate.
        turn_rate = gain * deviation

        # Set the drive base speed and turn rate.
        robot.drive(speed, flip * turn_rate)

        # You can wait for a short time or do other things in this loop.
        wait(delay)

        # TODO: figure out how far we just went
        gone_distance = robot.distance()



treadmill()
