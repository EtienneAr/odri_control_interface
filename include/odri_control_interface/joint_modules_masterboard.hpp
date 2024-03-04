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

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"
#include "odri_control_interface/joint_modules_abstract.hpp"


#include <odri_control_interface/common.hpp>

namespace odri_control_interface
{
/**
 * @brief Class abstracting the blmc motors to modules.
 */
class JointModulesMasterboard : public JointModulesAbstract
{
protected:
    std::shared_ptr<MasterBoardInterface> robot_if_;
    std::vector<Motor*> motors_;

    Eigen::VectorXd gear_ratios_;
    Eigen::VectorXd motor_constants_;
    VectorXi polarities_;

    // Cache for results.
    Eigen::VectorXd positions_;
    Eigen::VectorXd velocities_;
    Eigen::VectorXd sent_torques_;
    Eigen::VectorXd measured_torques_;

    // Cache for status bits.
    VectorXb index_been_detected_;
    VectorXb ready_;
    VectorXb enabled_;
    VectorXb motor_driver_enabled_;
    VectorXi motor_driver_errors_;

    int nd_;
public:
    JointModulesMasterboard(const std::shared_ptr<MasterBoardInterface>& robot_if,
                 ConstRefVectorXi motor_numbers,
                 double motor_constants,
                 double gear_ratios,
                 double max_currents,
                 ConstRefVectorXb reverse_polarities,
                 ConstRefVectorXd lower_joint_limits,
                 ConstRefVectorXd upper_joint_limits,
                 double max_joint_velocities,
                 double safety_damping);
    virtual ~JointModulesMasterboard() = default;

    void Enable() override;

    void ParseSensorData()  override;

    void SetTorques(ConstRefVectorXd desired_torques) override;
    void SetDesiredPositions(ConstRefVectorXd desired_positions) override;
    void SetDesiredVelocities(ConstRefVectorXd desired_velocities) override;
    void SetPositionGains(ConstRefVectorXd desired_gains) override;
    void SetVelocityGains(ConstRefVectorXd desired_gains) override;
    void SetMaximumCurrents(double max_currents);

    // Used for calibration.
    void SetPositionOffsets(ConstRefVectorXd position_offsets);

    void EnableIndexOffsetCompensation();
    void EnableIndexOffsetCompensation(int);

    const VectorXb& HasIndexBeenDetected();
    const VectorXb& GetReady();
    const VectorXb& GetEnabled();
    const VectorXb& GetMotorDriverEnabled();
    const VectorXi& GetMotorDriverErrors();

    bool SawAllIndices();

    /**
     * @brief Returns true once all motors are enabled and report ready.
     */
    bool IsReady() override;

    const VectorXd& GetPositions() override;
    const VectorXd& GetVelocities() override;
    const VectorXd& GetSentTorques() override;
    const VectorXd& GetMeasuredTorques() override;

    const VectorXd& GetGearRatios();


    /**
     * @brief Checks for errors and prints them
     */
    bool HasError()  override;

protected:
    int upper_joint_limits_counter_;
    int lower_joint_limits_counter_;
    int velocity_joint_limits_counter_;
    int motor_drivers_error_counter;
};

// For legacy purpose
using JointModules = JointModulesMasterboard;

}  // namespace odri_control_interface
