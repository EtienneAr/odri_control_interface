#! /usr/bin/env python
#
# Similar to `demo_solo12.py`. Uses the IMU attitude to control the desired
# joint positions.

import pinocchio as pin

import pathlib

import numpy as np
np.set_printoptions(suppress=True, precision=2)

import libodri_control_interface_pywrap as oci

# Create the robot object from yaml.
robot = oci.robot_from_yaml_file("config_solo12.yaml")

# Safety
robot.joints.set_maximum_current(8.0)

# Store initial position data.
init_imu_attitude = robot.imu.attitude_euler.copy()
des_pos = np.zeros(12)

Kp = 20.0
Kd = 0.1
Kd_off = 0.01

# Initialize the communication, session, joints, wait for motors to be ready
# and run the joint calibration.ArithmeticError()
robot.initialize(des_pos)



c = 0
while not robot.is_timeout and not robot.has_error:
    robot.parse_sensor_data()

    imu_attitude = robot.imu.attitude_euler
    positions = robot.joints.positions
    velocities = robot.joints.velocities
    torques = robot.joints.measured_torques

    # Left copy right
    # des_pos     = np.concatenate([  positions [3:6], np.zeros(3)       , positions [9:12], np.zeros(3)       ])
    # des_vel     = np.concatenate([  velocities[3:6], np.zeros(3)       , velocities[9:12], np.zeros(3)       ])
    # Kps         = np.concatenate([  Kp*np.ones(3)  , np.zeros(3)       , Kp*np.ones(3)   , np.zeros(3)       ])
    # Kds         = np.concatenate([  Kd*np.ones(3)  , Kd_off*np.ones(3) , Kd*np.ones(3)   , Kd_off*np.ones(3) ])
    # des_torques = np.concatenate([  np.zeros(3)    , -torques[0:3]     , np.zeros(3)     , -torques[6:9]     ])
    
    # Front copy back (move back)
    des_pos     = np.concatenate([  positions [6:9], positions [9:12], np.zeros(3)      , np.zeros(3)      ])
    des_vel     = np.concatenate([  velocities[6:9], velocities[9:12], np.zeros(3)      , np.zeros(3)      ])
    Kps         = np.concatenate([  Kp*np.ones(3)  , Kp*np.ones(3)   , np.zeros(3)      , np.zeros(3)      ])
    Kds         = np.concatenate([  Kd*np.ones(3)  , Kd*np.ones(3)   , Kd_off*np.ones(3), Kd_off*np.ones(3)])
    des_torques = np.concatenate([  np.zeros(3)    , np.zeros(3)     , -torques[0:3]    , -torques[3:6]    ])

    robot.joints.set_desired_positions(des_pos)
    robot.joints.set_desired_velocities(des_vel)
    robot.joints.set_torques(des_torques)
    robot.joints.set_position_gains(Kps)
    robot.joints.set_velocity_gains(Kds)

    robot.send_command_and_wait_end_of_cycle(0.001)
    c += 1

    if c % 500 == 0:
        print("IMU attitude:", imu_attitude)
        print("joint pos:   ", positions)
        print("joint vel:   ", velocities)
        print("HasError?:   ", robot.has_error)
        # print("torques:     ", torques)
        robot.robot_interface.PrintStats()

print("Final Error:   ", robot.has_error)
