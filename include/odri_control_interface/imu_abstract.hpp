/**
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-12-05
 *
 * @brief IMU abstraction.
 */

#pragma once

#include <odri_control_interface/common.hpp>

namespace odri_control_interface
{
/**
 * @brief Class for dealing with the IMU.
 */
class IMUAbstract
{
public:
    virtual bool HasError() = 0;
    virtual void ParseSensorData() = 0;

    virtual const Eigen::Vector3d& GetGyroscope() = 0;
    virtual const Eigen::Vector3d& GetAccelerometer() = 0;
    virtual const Eigen::Vector3d& GetLinearAcceleration() = 0;
    virtual const Eigen::Vector3d& GetAttitudeEuler() = 0;
    virtual const Eigen::Vector4d& GetAttitudeQuaternion() = 0;
};

}  // namespace odri_control_interface
