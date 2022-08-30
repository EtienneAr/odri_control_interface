#! /usr/bin/env python

import pinocchio as pin
import example_robot_data
from pinocchio.visualize import GepettoVisualizer

import pathlib

import numpy as np
np.set_printoptions(suppress=True, precision=2)

import libodri_control_interface_pywrap as oci
import threading

pin_robot = example_robot_data.load('solo12')
# viz = MeshcatVisualizer(pin_robot.model, pin_robot.collision_model, pin_robot.visual_model)
viz = GepettoVisualizer(pin_robot.model, pin_robot.collision_model, pin_robot.visual_model)
viz.initViewer(loadModel=True)

viz.display(pin_robot.q0)
input("Press any key to start")


# Create the robot object from yaml.
robot = oci.robot_from_yaml_file("config_solo12.yaml")

Kp = 2.0
Kd = 0.1

# Initialize the communication, session, joints, wait for motors to be ready
# and run the joint calibration.ArithmeticError()
robot.initialize(np.zeros(12))

c = 0
while not robot.is_timeout and not robot.has_error:
    robot.parse_sensor_data()

    imu_attitude_e = robot.imu.attitude_euler
    imu_attitude_q = robot.imu.attitude_quaternion
    positions = robot.joints.positions
    velocities = robot.joints.velocities

    robot.joints.set_zero_commands()

    robot.send_command_and_wait_end_of_cycle(0.001)

    c += 1

    if c % 100 == 0:
        print("IMU : ")
        robot.robot_interface.PrintIMU()
        print("")
        print("Robot attitude:", imu_attitude_e)
        print("Robot attitude:", imu_attitude_q)
        print("Robot accel:", robot.imu.accelerometer)
        print("joint pos:   ", positions)
        print("joint vel:   ", velocities)
        print("HasError?:   ", robot.has_error)
        # print("torques:     ", torques)
        robot.robot_interface.PrintStats()
        viz.display(np.concatenate([pin_robot.q0[:3], imu_attitude_q, positions]))
    


print("Final Error:   ", robot.has_error)
