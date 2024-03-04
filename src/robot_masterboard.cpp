/**
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-12-05
 *
 * @brief RobotMasterboard class orchistrating the devices.
 */

#include <stdexcept>   // for exception, runtime_error, out_of_range

#include "odri_control_interface/robot_masterboard.hpp"

namespace odri_control_interface
{

RobotMasterboard::RobotMasterboard(const std::shared_ptr<MasterBoardInterface>& robot_if,
          const std::shared_ptr<JointModulesMasterboard>& joint_modules,
          const std::shared_ptr<IMUMatserboard>& imu,
          const std::shared_ptr<JointCalibrator>& calibrator)
    : RobotAbstract<JointModulesMasterboard, IMUMatserboard>(joint_modules, imu),
    robot_if(robot_if),
    calibrator(calibrator)
    {}

const std::shared_ptr<MasterBoardInterface>& RobotMasterboard::GetRobotInterface()
{
    return robot_if;
}


void RobotMasterboard::Init()
{
    // Init the robot.
    robot_if->Init();

    // Enable the joints.
    joints->Enable();
}

void RobotMasterboard::SendInit()
{
    robot_if->SendInit();
}


void RobotMasterboard::Start()
{
    Init();

    // Initiate the communication session.
    std::chrono::time_point<std::chrono::system_clock> last =
        std::chrono::system_clock::now();
    while (!IsTimeout() && !IsAckMsgReceived())
    {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now() -
                                             last))
                .count() > 0.001)
        {
            last = std::chrono::system_clock::now();
            SendInit();
        }
    }

    if (IsTimeout())
    {
        throw std::runtime_error("Timeout during RobotMasterboard::Start().");
    }

    // Parse the sensor data to make sure all fields are filled properly when
    // the user starts using the robot object.
    ParseSensorData();
}


bool RobotMasterboard::IsAckMsgReceived()
{
    return IsAckMsgReceived();
}


bool RobotMasterboard::SendCommand()
{
    HasError();
    if (saw_error_)
    {
        joints->RunSafetyController();
    }
    SendCommand();
    return !saw_error_;
}


void RobotMasterboard::ParseSensorData()
{
    robot_if->ParseSensorData();
    joints->ParseSensorData();

    if (imu)
    {
        imu->ParseSensorData();
    }
}

bool RobotMasterboard::RunCalibration(VectorXd const& target_positions)
{
    return RunCalibration(calibrator, target_positions);
}

bool RobotMasterboard::RunCalibration(const std::shared_ptr<JointCalibrator>& calibrator,
                           VectorXd const& target_positions)
{
    bool is_done = false;
    if (target_positions.size() != joints->GetNumberMotors())
    {
        throw std::runtime_error(
            "Target position vector has a different size than the "
            "number of motors.");
    }
    while (!IsTimeout())
    {
        ParseSensorData();

        is_done = calibrator->RunAndGoTo(target_positions);

        if (is_done)
        {
            return true;
        }

        if (!SendCommandAndWaitEndOfCycle(calibrator->dt()))
        {
            throw std::runtime_error("Error during RobotMasterboard::RunCalibration().");
        }
    }

    throw std::runtime_error("Timeout during RobotMasterboard::RunCalibration().");
    return false;
}

/**
 * @brief Returns true if all connected devices report ready.
 */
bool RobotMasterboard::IsReady()
{
    return joints->IsReady();
}


bool RobotMasterboard::WaitUntilReady()
{
    ParseSensorData();
    joints->SetZeroCommands();

    std::chrono::time_point<std::chrono::system_clock> last =
        std::chrono::system_clock::now();
    while (!IsReady() && !HasError())
    {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now() -
                                             last))
                .count() > 0.001)
        {
            last += std::chrono::milliseconds(1);
            if (!IsAckMsgReceived()) {
                SendInit();
            } else {
                ParseSensorData();
                SendCommand();
            }
        }
        else
        {
            std::this_thread::yield();
        }
    }

    if (HasError()) {
        if (IsTimeout())
        {
            throw std::runtime_error("Timeout during RobotMasterboard::WaitUntilReady().");
        } else {
            throw std::runtime_error("Error during RobotMasterboard::WaitUntilReady().");
        }
    }

    return !saw_error_;
}

void RobotMasterboard::Initialize(VectorXd const& target_positions)
{
    Start();
    WaitUntilReady();
    RunCalibration(target_positions);
}

bool RobotMasterboard::IsTimeout()
{
    return robot_if->IsTimeout();
}

bool RobotMasterboard::RobotHasError_()
{
    if (robot_if->IsTimeout())
    {
        if (timeout_counter_++ % 2000 == 0)
        {
            msg_out_ << "ERROR: Robot communication timedout." << std::endl;
        }
        return true;
    }
    return false;
}

}  // namespace odri_control_interface
