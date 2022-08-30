#! /usr/bin/env python
#
# Similar to `demo_solo12.py`. Uses the IMU attitude to control the desired
# joint positions.

import pinocchio as pin
import example_robot_data
from pinocchio.visualize import MeshcatVisualizer

import pathlib

import numpy as np
np.set_printoptions(suppress=True, precision=2)

import libodri_control_interface_pywrap as oci
import threading

pin_robot = example_robot_data.load('solo12')
viz = MeshcatVisualizer(pin_robot.model, pin_robot.collision_model, pin_robot.visual_model)
viz.initViewer(loadModel=True, open=True)

# Create the robot object from yaml.
robot = oci.robot_from_yaml_file("config_solo12.yaml")

# Store initial position data.
init_imu_attitude = robot.imu.attitude_euler.copy()
des_pos = np.zeros(12)

Kp = 2.0
Kd = 0.1

# Initialize the communication, session, joints, wait for motors to be ready
# and run the joint calibration.ArithmeticError()
robot.initialize(des_pos)


def go_q(q):
    viz.display(np.concatenate([pin_robot.q0[:7], q]))

    def get_input():
        keystrk = input()
        # thread doesn't continue until key is pressed
        # and so it remains alive

    c = 0
    th = threading.Thread(target=get_input)
    th.start()
    while not robot.is_timeout and not robot.has_error and th.is_alive():
        robot.parse_sensor_data()

        imu_attitude = robot.imu.attitude_euler
        positions = robot.joints.positions
        velocities = robot.joints.velocities

        #des_pos[:] = imu_attitude[2] - init_imu_attitude[2]
        # torques = Kp * (q - positions) - Kd * velocities
        robot.joints.set_torques(np.zeros(12))
        robot.joints.set_desired_positions(q)
        robot.joints.set_desired_velocities(np.zeros(12))
        robot.joints.set_position_gains(Kp*np.ones(12))
        robot.joints.set_velocity_gains(Kd*np.ones(12))

        robot.send_command_and_wait_end_of_cycle(0.001)
        c += 1

        if c % 500 == 0:
            print("IMU attitude:", imu_attitude)
            print("joint pos:   ", positions)
            print("joint vel:   ", velocities)
            print("HasError?:   ", robot.has_error)
            # print("torques:     ", torques)
            robot.robot_interface.PrintStats()

for i in range(12):
    q = np.copy(des_pos)
    q[i] += 3.141592 / 10.
    go_q(q)

# go_q(np.array([0, np.pi/4, -np.pi/2]*4))

print("Final Error:   ", robot.has_error)
