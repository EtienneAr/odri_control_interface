/**
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-11-27
 *
 * @brief Joint module abstraction for
 */

#pragma once

#include <unistd.h>
#include <iostream>
#include <vector>
#include <memory>

#include <odri_control_interface/common.hpp>

namespace odri_control_interface
{
/**
 * @brief Class abstracting the blmc motors to modules.
 */
class JointModulesAbstract
{
protected:
    Eigen::VectorXd lower_joint_limits_;
    Eigen::VectorXd upper_joint_limits_;
    Eigen::VectorXd safety_damping_;

    Eigen::VectorXd zero_vector_;
    double max_joint_velocities_;
    double max_joint_current_;

    int n_;

    bool check_joint_limits_;

    std::ostream& msg_out_ = std::cout;

public:
    JointModulesAbstract(double max_currents,
                 ConstRefVectorXd lower_joint_limits,
                 ConstRefVectorXd upper_joint_limits,
                 double max_joint_velocities,
                 double safety_damping);
    virtual ~JointModulesAbstract() = default;

    virtual void Enable() = 0;

    virtual void ParseSensorData() = 0;

    virtual void SetTorques(ConstRefVectorXd desired_torques) = 0;
    virtual void SetDesiredPositions(ConstRefVectorXd desired_positions) = 0;
    virtual void SetDesiredVelocities(ConstRefVectorXd desired_velocities) = 0;
    virtual void SetPositionGains(ConstRefVectorXd desired_gains) = 0;
    virtual void SetVelocityGains(ConstRefVectorXd desired_gains) = 0;

    /**
     * @brief Disables the position and velocity gains by setting them to zero.
     */
    void SetZeroGains();
    void SetZeroCommands();

    /**
     * @brief Overwrites the control commands for a default safety controller.
     * The safety controller applies a D control to all the joints based
     * on the provided `safety_damping`.
     */
    virtual void RunSafetyController();

    /**
     * @brief Returns true once all motors are enabled and report ready.
     */
    virtual bool IsReady() = 0;

    virtual const VectorXd& GetPositions() = 0;
    virtual const VectorXd& GetVelocities() = 0;
    virtual const VectorXd& GetSentTorques() = 0;
    virtual const VectorXd& GetMeasuredTorques() = 0;

    int GetNumberMotors();

    void DisableJointLimitCheck();
    void EnableJointLimitCheck();

    /**
     * @brief Checks for errors and prints them
     */
    virtual bool HasError() = 0;

    void PrintVector(ConstRefVectorXd vector);

protected:
    int upper_joint_limits_counter_;
    int lower_joint_limits_counter_;
    int velocity_joint_limits_counter_;
};

}  // namespace odri_control_interface
