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


#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

#include <odri_control_interface/calibration.hpp>
#include <odri_control_interface/imu_masterboard.hpp>
#include <odri_control_interface/joint_modules_masterboard.hpp>
#include <odri_control_interface/robot_abstract.hpp>
#include <odri_control_interface/robot_masterboard.hpp>

namespace odri_control_interface
{

class RobotMasterboard : public RobotAbstract<JointModulesMasterboard, IMUMatserboard>
{
public:
    std::shared_ptr<MasterBoardInterface> robot_if;
    std::shared_ptr<JointCalibrator> calibrator;

public:
    RobotMasterboard(const std::shared_ptr<MasterBoardInterface>& robot_if,
          const std::shared_ptr<JointModulesMasterboard>& joint_modules,
          const std::shared_ptr<IMUMatserboard>& imu,
          const std::shared_ptr<JointCalibrator>& calibrator);

    ~RobotMasterboard() = default;

    /**
     * @brief Returns the underlying robot interface
     */
    const std::shared_ptr<MasterBoardInterface>& GetRobotInterface();

    /**
     * @brief Initializes the connection. Use `SendInit` to initialize the
     * session.
     */
    void Init();

    /**
     * @brief Establishes the session with the robot interface.
     */
    void SendInit();

    /**
     * @brief Initializes the session and blocks until either the package
     *   got acknowledged or the communication timed out.
     */
    void Start();

    /**
    * @copydoc RobotAbstract::Initialize
    */
    void Initialize(VectorXd const& target_positions) override;

    /**
    * @copydoc RobotAbstract::SendCommand
    */
    bool SendCommand() override;

    /**
    * @copydoc::ParseSensorData RobotAbstract
    */
    void ParseSensorData() override;

    /**
     * @brief Run the calibration procedure and blocks. Returns true if the
     * calibration procedure finished successfully. Otherwise (e.g. when an
     * error occurred or the communication timed-out) return false.
     */
    bool RunCalibration(const std::shared_ptr<JointCalibrator>& calibrator,
                        VectorXd const& target_positions);

    /**
     * @brief Runs the calibration procedure for the calibrator passed in
     * during initialization and blocks. * calibration procedure finished
     * successfully. Otherwise (e.g. when an error occurred or the communication
     * timed-out) return false.
     */
    bool RunCalibration(VectorXd const& target_positions);

    /**
    * @copydoc RobotAbstract::IsReady
    */
    bool IsReady() override;

    /**
     * @brief Returns true if the connection timed out.
     */
    bool IsTimeout();

    /**
     * @brief Returns true if during the session initialization the message got
     * acknowledged.
     */
    bool IsAckMsgReceived();

    /**
    * @copydoc RobotAbstract::WaitUntilReady
    */
    bool WaitUntilReady() override;

protected:
    /**
     * @brief Robot specific implementation for error checking
     */
    bool RobotHasError_() override;
};

// For legacy purpose
using Robot = RobotMasterboard;

}  // namespace odri_control_interface
