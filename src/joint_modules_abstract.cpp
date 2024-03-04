/**
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-11-27
 *
 * @brief Joint module abstraction for
 */

#include "odri_control_interface/joint_modules_abstract.hpp"

namespace odri_control_interface
{
JointModulesAbstract::JointModulesAbstract(
    double max_currents,
    ConstRefVectorXd lower_joint_limits,
    ConstRefVectorXd upper_joint_limits,
    double max_joint_velocities,
    double safety_damping)
    :
      lower_joint_limits_(lower_joint_limits),
      upper_joint_limits_(upper_joint_limits),
      max_joint_velocities_(max_joint_velocities),
      max_joint_current_(max_currents),
      check_joint_limits_(true),
      upper_joint_limits_counter_(0),
      lower_joint_limits_counter_(0),
      velocity_joint_limits_counter_(0)
{
    // Check input arrays for correct sizes.
    if (lower_joint_limits.size() != upper_joint_limits.size())
    {
        throw std::runtime_error("Lower joint limits has different size than upper joint limits");
    }

    n_ = static_cast<int>(lower_joint_limits.size());

    // Resize and fill the vectors.
    zero_vector_.resize(n_);
    zero_vector_.fill(0.);
    safety_damping_.resize(n_);
    safety_damping_.fill(safety_damping);
}

int JointModulesAbstract::GetNumberMotors()
{
    return n_;
}

void JointModulesAbstract::SetZeroGains()
{
    SetPositionGains(zero_vector_);
    SetVelocityGains(zero_vector_);
}

void JointModulesAbstract::SetZeroCommands()
{
    SetTorques(zero_vector_);
    SetDesiredPositions(zero_vector_);
    SetDesiredVelocities(zero_vector_);
    SetZeroGains();
}

void JointModulesAbstract::RunSafetyController()
{
    SetZeroCommands();
    SetVelocityGains(safety_damping_);
}

void JointModulesAbstract::DisableJointLimitCheck()
{
    check_joint_limits_ = false;
}

void JointModulesAbstract::EnableJointLimitCheck()
{
    check_joint_limits_ = true;
}

void JointModulesAbstract::PrintVector(ConstRefVectorXd vector)
{
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    msg_out_ << vector.transpose().format(CleanFmt);
}

}  // namespace odri_control_interface
