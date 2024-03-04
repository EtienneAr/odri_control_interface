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

#include <unistd.h>

#include <odri_control_interface/imu_abstract.hpp>
#include <odri_control_interface/joint_modules_abstract.hpp>

namespace odri_control_interface
{
/**
 * @brief Class abstracting the control and sensor read.
 */
template<class JointModules, class IMU>
class RobotAbstract
{
static_assert(std::is_base_of<JointModulesAbstract, JointModules>::value, "JointModules must derive from JointModulesAbstract");
static_assert(std::is_base_of<IMUAbstract, IMU>::value, "IMU must derive from IMUAbstract");

public:
    std::shared_ptr<JointModules> joints;
    std::shared_ptr<IMU> imu;

protected:
    int timeout_counter_;
    bool saw_error_;
    std::ostream& msg_out_ = std::cout;
    std::chrono::time_point<std::chrono::system_clock> last_time_;

public:
    RobotAbstract(const std::shared_ptr<JointModules>& joint_modules, const std::shared_ptr<IMU>& imu)
        : joints(joint_modules),
          imu(imu),
          last_time_(std::chrono::system_clock::now())
    {}

    virtual ~RobotAbstract() = default;

    /**
     * @brief Returns the joint module.
     */
    const std::shared_ptr<JointModules>& GetJoints();

    /**
     * @brief Return the IMU.
     */
    const std::shared_ptr<IMU>& GetIMU();

    /**
     * @brief Initializes the robot. This inclues establishing the communication
     * to the robot, wait until the joints are all ready, running the
     * calibration procedure and waiting in the desired initial configuration.
     */
    virtual void Initialize(VectorXd const& target_positions) = 0;

    /**
     * @brief If no error happend, send the previously specified commands
     *   to the robot. If an error was detected, go into safety mode
     *   and apply the safety control from the joint_module.
     */
    virtual bool SendCommand() = 0;

    /**
     * @brief Same as SendCommand but waits till the end of the control cycle.
     */
    bool SendCommandAndWaitEndOfCycle(double dt);

    /**
     * @brief Parses the sensor data and calls ParseSensorData on all devices.
     */
    virtual void ParseSensorData() = 0;

    /**
     * @brief Returns true if all connected devices report ready.
     */
    virtual bool IsReady() = 0;

    /**
     * @brief Blocks until all devices report ready.
     */
    virtual bool WaitUntilReady() = 0;

    /**
     * @brief Checks all connected devices for errors.
     */
    bool HasError();

    /**
     * @brief Way to report an external error. Causes the robot to go into
     *   safety mode.
     */
    void ReportError(const std::string& error);

    /**
     * @brief Way to report an external error quietly. Causes the robot to go into
     *   safety mode.
     */
    void ReportError();

protected:
    /**
     * @brief Robot specific implementation for error checking
     */
    virtual bool RobotHasError_() = 0;

};

}  // namespace odri_control_interface

#include "odri_control_interface/robot_abstract.hxx"
