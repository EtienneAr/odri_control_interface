/**
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-11-27
 *
 * @brief Main robot interface from the package.
 */

#pragma once

#include "odri_control_interface/robot_abstract.hpp"

namespace odri_control_interface
{

template<class JointModules, class IMU>
const std::shared_ptr<JointModules>& RobotAbstract<JointModules, IMU>::GetJoints()
{
    return joints;
}

template<class JointModules, class IMU>
const std::shared_ptr<IMU>& RobotAbstract<JointModules, IMU>::GetIMU()
{
    return imu;
}

template<class JointModules, class IMU>
bool RobotAbstract<JointModules, IMU>::SendCommandAndWaitEndOfCycle(double dt)
{
    bool result = SendCommand();

    while (((std::chrono::duration<double>)(std::chrono::system_clock::now() -
                                            last_time_))
               .count() < dt)
    {
        std::this_thread::yield();
    }
    last_time_ = std::chrono::system_clock::now();

    return result;
}

template<class JointModules, class IMU>
void RobotAbstract<JointModules, IMU>::ReportError(const std::string& error)
{
    msg_out_ << "ERROR: " << error << std::endl;
    ReportError();
}

template<class JointModules, class IMU>
void RobotAbstract<JointModules, IMU>::ReportError()
{
    saw_error_ = true;
}


template<class JointModules, class IMU>
bool RobotAbstract<JointModules, IMU>::HasError()
{
    saw_error_ |= joints->HasError();
    if (imu)
    {
        saw_error_ |= imu->HasError();
    }

    saw_error_ |= RobotHasError_();

    return saw_error_;
}

}  // namespace odri_control_interface
