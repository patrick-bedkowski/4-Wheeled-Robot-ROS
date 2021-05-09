#!/usr/bin/env python3
# encoding: utf8

import rospy
import turtlesim
from turtlesim.msg import Pose
from turtlesim.srv import SetPenRequest
from TurtlesimSIU import TurtlesimSIU
import math
import numpy as np
from pynput import keyboard  # module that reads input from keyboard
import sys
import signal
import csv  # for saving data to csv file
import argparse
from turtlebotClass import TurtleBot

active_key = None
keys_queue = []
lin_v = None
ang_v = None

control_keys = ['w', 's', 'a', 'd', 'Space']  # list of defined control keys

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('vl', help='Linear velocity of the robot')
parser.add_argument('vr', help='Angular velocity of the robot')
args = parser.parse_args()

linear_velocity = float(args.vl)
angular_velocity = float(args.vr)


def active_key_apply():
    global lin_v
    global ang_v

    if active_key == 'w':
        lin_v = linear_velocity
        ang_v = 0
    elif active_key == 's':
        lin_v = -linear_velocity
        ang_v = 0
    elif active_key == 'a':
        lin_v = 0
        ang_v = angular_velocity
    elif active_key == 'd':
        lin_v = 0
        ang_v = -angular_velocity
    elif active_key == 'Space' or active_key is None:
        lin_v = 0
        ang_v = 0


def on_press(key):
    try:
        key = key.char
    except AttributeError:
        if key == keyboard.Key.space:
            key = 'Space'
        else:
            key = None

    if key in control_keys:
        global active_key
        global keys_queue

        if not active_key:
            active_key = key
            active_key_apply()
        elif key != active_key and key not in keys_queue:
            if key == 'Space':
                active_key = key
                active_key_apply()
                keys_queue.clear()
            else:
                keys_queue.append(key)


def on_release(key):
    try:
        key = key.char
    except AttributeError:
        if key == keyboard.Key.space:
            key = 'Space'
        else:
            key = None

    if key in control_keys:
        global active_key
        global keys_queue

        if key == active_key:
            active_key = None
            active_key_apply()
            if keys_queue:
                active_key = keys_queue.pop(0)
                active_key_apply()
        elif key in keys_queue:
            keys_queue.remove(key)


def signal_handler(sig, frame):
    print("\nTerminating")
    sys.exit(0)


def log_turtles_positions(robot, file):
    '''Logs turtles positions to csv file'''
    writer = csv.writer(file)
    writer.writerow(robot.turtles_positions())
    return robot.turtles_positions()


if __name__ == "__main__":

    # open csv file
    log_file = open('data_310603_310893.csv', mode='w', newline='')

    # create a ROBOT
    robot = TurtleBot()

    # listen for keyboard inputs
    signal.signal(signal.SIGINT, signal_handler)
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

    # executes until the ros has been shut down
    while not rospy.is_shutdown():
        # CHECK COLISIONS WITH WALLS
        robot.check_colisions()

        # print position
        # linear_velocity = 0
        # angular_velocity = 1
        # print(f'ANGULAR: {angular_velocity}\n')
        # print(f'Linear: {linear_velocity}\n')
        if lin_v == 0 and ang_v != 0:
            # print('-------------------ROTATE------------------------')
            # print(robot.position_of_each_turtle())
            robot.rotate_robot(ang_v)
            log_turtles_positions(robot, log_file)
            # robot.pose.theta = robot.angular_v
        elif ang_v == 0 and lin_v != 0:
            # print('--------------------MOVE-------------------------')
            # print(robot.position_of_each_turtle())
            robot.move_robot(lin_v)
            log_turtles_positions(robot, log_file)

        robot.rate.sleep()


'''
            =============================
            |       TurtleBot CLASS     |
            =============================
'''

class TurtleBot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('siu_example', anonymous=False)  # initiate node
        self.turtle_api = TurtlesimSIU.TurtlesimSIU()
        self.hz = 30
        self.rate = rospy.Rate(self.hz)
        self.time = 1/self.hz  # time correction

        self.colision = False

        # Robot parameters
        self._length = 6  # robot length
        self._width = 4.5  # robot width
        self.angle = 0

        # Window Size
        self.window_size = self.turtle_api.getFrameSize()

        # constant value dependant on the parameters
        # 0,6435 rad
        self._rotation_angle = math.atan(((self._width/2)/(self._length/2)))
        # it indicates angle between direction angle of the central turtle
        # and displacement angle from central turtle to the front left turtle

        # Reset kinematic data
        self.delta_x = 0
        self.delta_y = 0
        self.liniear_v_resultant = 0
        self.angular_v = 0

        # displacement vector - vector from center turtle to the front left one
        self.displacement_vector = np.array(
            [self._length / 2, self._width / 2]
        )

        # holds fixed displacement vectors of each turtle
        self.turtles_vectors = {
            'front_left': self.displacement_vector,
            'front_right': self.rotate_vector(
                self.displacement_vector, (-2 * self._rotation_angle)
            ),
            'back_right': self.rotate_vector(
                self.displacement_vector, math.pi
            ),
            'back_left': self.rotate_vector(self.displacement_vector, (
                math.pi - 2 * self._rotation_angle)
            ),
            'central': [0, 0]
        }

        # holds current absolute position of all turtles
        self.current_turtles_position = {
            'front_left': [0, 0],
            'front_right': [0, 0],
            'back_right': [0, 0],
            'back_left': [0, 0],
            'central': [0, 0]
        }

        self.spawn_robot(12, 7, 0)  # spawn robot
        self.set_pens()  # set pens of the turtles

        # Data of central turtle
        self.pose = self.turtle_api.getPose('central')

        # print settings data
        print(f'Pixel/smth scale: {self.turtle_api.pixelsToScale()}')
        print(f'Window size: \n > width: {self.window_size.width} [meters]\n > height: {self.window_size.height} [meters]')


    '''
            ==================================
            |       SPAWN AND SET PENS       |
            ==================================
    '''

    def spawn_turtle(self, name, x1, y1, angle):
        '''Spawns single turtle'''
        if self.turtle_api.hasTurtle(name):
            self.turtle_api.killTurtle(name)
            rospy.sleep(2)
        if not self.turtle_api.hasTurtle(name):
            self.turtle_api.spawnTurtle(
                name, turtlesim.msg.Pose(x=x1, y=y1, theta=angle)
            )

    def spawn_robot(self, x1, y1, angle):
        '''Inserted coordinates are
        coordinates of the central turtle '''
        for turtle_name, position in self.turtles_vectors.items():
            x = x1 + position[0]
            y = y1 + position[1]

            self.spawn_turtle(name=turtle_name, x1=x, y1=y, angle=angle)

            # updating position
            self.current_turtles_position[turtle_name] = [x, y]

    def set_pens(self):
        ''' Sets pen of each turtle '''
        pen_center_turtle = turtlesim.srv.SetPenRequest(
            r=255, g=255, b=255, width=5, off=0
        )
        pen_side_turtles = turtlesim.srv.SetPenRequest(
            r=50, g=50, b=255, width=3, off=0
        )

        # iterate through available turtles
        for turtle in self.turtles_vectors.keys():
            if turtle == 'central':  # central turtle has different pen
                self.turtle_api.setPen('central', pen_center_turtle)
                continue
            self.turtle_api.setPen(turtle, pen_side_turtles)

    def turtles_positions(self):
        '''Returns information about absolute positions of all turtles'''

        positions = []
        for turtle_name, position in self.current_turtles_position.items():
            x = position[0]
            y = position[1]
            angle = self.angle
            positions += [x, y, angle]
            # positions += f'{turtle_name} x: {position[0]}, y: {position[1]} '
            # positions += f'Angle: {self.angle}\n'
        return positions

    '''
            =========================
            |       Kinematics      |
            =========================
    '''
    def calculate_partial_velocities(self, linear_velocity):
        '''Calculates vertical and horizontal velocities'''
        x_v = linear_velocity * np.cos(self.angle)
        y_v = linear_velocity * np.sin(self.angle)
        return x_v, y_v

    def calculate_delta_coordinates(self, x_v, y_v):
        '''Calculates change of coorinates in time'''

        delta_x = x_v * self.time
        delta_y = y_v * self.time
        return delta_x, delta_y

    '''
            =========================
            |       COLISIONS      |
            =========================
    '''

    '''
    We don't use getColisions() method, because it prints unwanted data
    to console, and slows simulation
    Thus, we implemented our own function to check if there are any colisions.
    '''

    def check_colisions(self):
        '''Returns boolean value True if colision happens'''

        # iterate through turtles and theirs coordinates
        for turtle_name, position in self.current_turtles_position.items():
            x = position[0]  # get coordinate
            y = position[1]

            if (x < 0) or (y < 0) or (x > self.window_size.width) or (y > self.window_size.height):
                self.colision = True  # set flag
                print('===============COLISION===============')
            else:
                self.colision = False  # set flag

    '''
            =========================
            |        ROTATION       |
            =========================
    '''

    def rotate_vector(self, vector, delta_angle):
        '''Returns rotated vector by input angle in radians'''
        r_matrix = np.array(
            [
                [math.cos(delta_angle), -math.sin(delta_angle)],
                [math.sin(delta_angle), math.cos(delta_angle)]
            ]
        )
        rotated_vector = np.dot(r_matrix, vector)
        return rotated_vector

    def rotate_turtle(self, turtle_name, delta_angle):
        '''Returns new vector rotated by the inserted angle in radians.
        It is a vector of displacement in reference to the center turtle'''

        self.turtles_vectors[turtle_name] = self.rotate_vector(
            self.turtles_vectors[turtle_name], delta_angle
        )
        return self.turtles_vectors[turtle_name]

    def next_coordinates(self, turtle_name, delta_angle):
        ''' Returns x, y coordinates of the next point on
        a rotation circle that is rotate by the input angle in radians'''

        # rotate displacement vector
        new_vector = self.rotate_turtle(turtle_name, delta_angle)

        # convert to absolute Cartesian system coordinates
        new_vector = np.add(
            new_vector, np.array(
                [
                    self.current_turtles_position['central'][0],
                    self.current_turtles_position['central'][1]
                ]
            )
        )

        return new_vector[0], new_vector[1]

    def rotate_robot(self, angular_velocity):
        '''Rotates robot with respect to given angular velocity'''

        delta_angle = angular_velocity * self.time
        self.angle += delta_angle  # increase absolute angle

        # set position of the central turtle
        self.turtle_api.setPose(turtle_name='central', pose=turtlesim.msg.Pose(
            x=self.current_turtles_position['central'][0],
            y=self.current_turtles_position['central'][1],
            theta=self.angle
        ), mode='absolute')

        for turtle_name in [
                    'front_left', 'front_right', 'back_right', 'back_left'
                ]:  # iterate through turtles

            # calculate next coordinates after rotation by angle
            x1, y1 = self.next_coordinates(turtle_name, delta_angle)

            # set position of the side turtle
            self.turtle_api.setPose(
                turtle_name=turtle_name, pose=turtlesim.msg.Pose(
                    x=x1,
                    y=y1,
                    theta=self.angle
                ), mode='absolute'
            )

            # update absolute position
            self.current_turtles_position[turtle_name] = [x1, y1]

    '''
            ===========================
            |        MOVE ROBOT       |
            ===========================
    '''

    def move_robot(self, linear_velocity):
        '''Moves robot with respect to given linear velocity'''

        x_v, y_v = self.calculate_partial_velocities(linear_velocity)

        delta_x, delta_y = self.calculate_delta_coordinates(x_v, y_v)

        # iterate through turtle names and its positions
        for turtle_name, position in self.current_turtles_position.items():
            # update position of current turtle
            self.current_turtles_position[turtle_name][0] += delta_x
            self.current_turtles_position[turtle_name][1] += delta_y

            self.turtle_api.setPose(  # set new position
                turtle_name=turtle_name, pose=turtlesim.msg.Pose(
                    x=self.current_turtles_position[turtle_name][0],
                    y=self.current_turtles_position[turtle_name][1],
                    theta=self.angle
                ), mode='absolute'
            )
