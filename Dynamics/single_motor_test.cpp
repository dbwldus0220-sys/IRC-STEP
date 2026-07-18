// Build examples:
//   Read only: g++ -std=c++17 single_motor_test.cpp -o single_motor_test \
//     -DREAD_ONLY=1 -L/usr/local/lib -ldxl_x64_cpp -pthread
//   ID scan: g++ -std=c++17 single_motor_test.cpp -o single_motor_scan \
//     -DID_SCAN=1 -L/usr/local/lib -ldxl_x64_cpp -pthread
//   Small move: g++ -std=c++17 single_motor_test.cpp -o single_motor_test \
//     -DTEST_ID=17 -DMOVE_TICK=10 \
//     -L/usr/local/lib -ldxl_x64_cpp -pthread

#include "dynamixel.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#ifndef TEST_ID
#define TEST_ID 17
#endif

#ifndef MOVE_TICK
#define MOVE_TICK 10
#endif

#ifndef READ_ONLY
#define READ_ONLY 0
#endif

#ifndef ID_SCAN
#define ID_SCAN 0
#endif

#if READ_ONLY && ID_SCAN
#error "READ_ONLY and ID_SCAN cannot both be enabled"
#endif

namespace
{
constexpr int kWaitMs = 1000;
constexpr int32_t kMinPosition = 0;
constexpr int32_t kMaxPosition = 4095;

bool IsPositionInRange(int32_t position)
{
    return position >= kMinPosition && position <= kMaxPosition;
}

struct LegMotor
{
    uint8_t id;
    const char* joint_name;
};

constexpr std::array<LegMotor, 12> kLegMotors{{
    {10, "right_hip_yaw_joint"},
    {13, "right_hip_roll_joint"},
    {15, "right_hip_pitch_joint"},
    {17, "right_knee_pitch_joint"},
    {19, "right_ankle_pitch_joint"},
    {21, "right_ankle_roll_joint"},
    {12, "left_hip_yaw_joint"},
    {14, "left_hip_roll_joint"},
    {16, "left_hip_pitch_joint"},
    {18, "left_knee_pitch_joint"},
    {20, "left_ankle_pitch_joint"},
    {22, "left_ankle_roll_joint"},
}};

const LegMotor* FindLegMotor(int id)
{
    for (const LegMotor& motor : kLegMotors)
    {
        if (motor.id == id)
        {
            return &motor;
        }
    }
    return nullptr;
}

bool ReadPresentPosition(
    dynamixel::PortHandler* port_handler,
    dynamixel::PacketHandler* packet_handler,
    uint8_t id,
    int32_t& position)
{
    uint8_t dxl_error = 0;
    uint32_t raw_position = 0;
    const int result = packet_handler->read4ByteTxRx(
        port_handler,
        id,
        DxlReg_PresentPosition,
        &raw_position,
        &dxl_error
    );
    if (result != COMM_SUCCESS || dxl_error != 0)
    {
        return false;
    }
    position = static_cast<int32_t>(raw_position);
    return true;
}

bool WriteTorque(
    dynamixel::PortHandler* port_handler,
    dynamixel::PacketHandler* packet_handler,
    uint8_t id,
    bool enable)
{
    uint8_t dxl_error = 0;
    const int result = packet_handler->write1ByteTxRx(
        port_handler,
        id,
        DxlReg_TorqueEnable,
        enable ? 1 : 0,
        &dxl_error
    );
    return result == COMM_SUCCESS && dxl_error == 0;
}

bool WriteGoalPosition(
    dynamixel::PortHandler* port_handler,
    dynamixel::PacketHandler* packet_handler,
    uint8_t id,
    int32_t position)
{
    uint8_t dxl_error = 0;
    const int result = packet_handler->write4ByteTxRx(
        port_handler,
        id,
        DxlReg_GoalPosition,
        static_cast<uint32_t>(position),
        &dxl_error
    );
    return result == COMM_SUCCESS && dxl_error == 0;
}

class PortCloseGuard
{
public:
    PortCloseGuard(dynamixel::PortHandler* port_handler, bool& port_open)
        : port_handler_(port_handler), port_open_(port_open)
    {
    }

    ~PortCloseGuard()
    {
        if (port_open_)
        {
            port_handler_->closePort();
        }
    }

private:
    dynamixel::PortHandler* port_handler_;
    bool& port_open_;
};
}  // namespace

int main()
{
    constexpr int test_id_value = TEST_ID;
    constexpr int32_t move_tick = static_cast<int32_t>(MOVE_TICK);
    constexpr bool read_only = READ_ONLY != 0;
    constexpr bool id_scan = ID_SCAN != 0;

    static_assert(MOVE_TICK >= -30 && MOVE_TICK <= 30,
                  "|MOVE_TICK| must not exceed 30");

    const LegMotor* test_motor = FindLegMotor(test_id_value);
    if (!id_scan && test_motor == nullptr)
    {
        std::cerr << "[ERROR] TEST_ID is not in the leg-motor allowlist."
                  << std::endl;
        return 1;
    }

    auto* port_handler =
        dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    auto* packet_handler =
        dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    bool port_open = false;
    PortCloseGuard port_close_guard(port_handler, port_open);
    bool torque_enabled = false;
    bool torque_was_enabled = false;
    bool return_attempted = false;
    bool return_succeeded = false;
    bool torque_disable_attempted = false;
    bool torque_disable_succeeded = false;
    bool start_position_available = false;
    bool target_position_available = false;
    int32_t start_position = 0;
    int32_t target_position = 0;
    int exit_code = 1;

    std::cout << "[MODE] "
              << (id_scan ? "ID_SCAN" : read_only ? "READ_ONLY" : "MOVE")
              << std::endl;
    if (!id_scan)
    {
        std::cout << "[INFO] TEST_ID          : " << test_id_value << std::endl;
        std::cout << "[INFO] joint_name       : "
                  << test_motor->joint_name << std::endl;
        std::cout << "[INFO] MOVE_TICK        : " << move_tick << std::endl;
    }

    try
    {
        if (!port_handler->openPort())
        {
            std::cerr << "[ERROR] Failed to open port: " << DEVICE_NAME
                      << std::endl;
        }
        else
        {
            port_open = true;
            if (!port_handler->setBaudRate(BAUDRATE))
            {
                std::cerr << "[ERROR] Failed to set baudrate: " << BAUDRATE
                          << std::endl;
            }
            else if (id_scan)
            {
                bool all_responded = true;
                for (const LegMotor& motor : kLegMotors)
                {
                    int32_t position = 0;
                    const bool responded = ReadPresentPosition(
                        port_handler,
                        packet_handler,
                        motor.id,
                        position
                    );
                    std::cout << "[SCAN] ID=" << static_cast<int>(motor.id)
                              << " joint=" << motor.joint_name
                              << " status="
                              << (responded ? "RESPONDED" : "NO_RESPONSE");
                    if (responded)
                    {
                        std::cout << " present_position=" << position;
                    }
                    std::cout << std::endl;
                    all_responded = all_responded && responded;
                }
                exit_code = all_responded ? 0 : 2;
            }
            else if (!ReadPresentPosition(
                         port_handler,
                         packet_handler,
                         test_motor->id,
                         start_position))
            {
                std::cerr << "[ERROR] Failed to read present position."
                          << std::endl;
            }
            else
            {
                start_position_available = true;
                std::cout << "[INFO] start_position   : " << start_position
                          << std::endl;

                if (!IsPositionInRange(start_position))
                {
                    std::cerr
                        << "[ERROR] Present position is outside [0, 4095]."
                        << std::endl;
                }
                else if (read_only)
                {
                    // READ_ONLY exits without torque or goal-position writes.
                    exit_code = 0;
                }
                else
                {
                    target_position = start_position + move_tick;
                    target_position_available = true;
                    std::cout << "[INFO] target_position  : "
                              << target_position << std::endl;

                    if (!IsPositionInRange(target_position))
                    {
                        std::cerr
                            << "[ERROR] Target position is outside [0, 4095]."
                            << std::endl;
                    }
                    else
                    {
                        std::string confirmation;
                        std::cout << "Type YES to continue" << std::endl;
                        std::getline(std::cin, confirmation);

                        if (confirmation != "YES")
                        {
                            std::cerr
                                << "[ABORT] First confirmation failed."
                                << std::endl;
                        }
                        else
                        {
                            std::cout << "Type MOTOR_ON to enable torque"
                                      << std::endl;
                            std::getline(std::cin, confirmation);

                            if (confirmation != "MOTOR_ON")
                            {
                                std::cerr
                                    << "[ABORT] Second confirmation failed."
                                    << std::endl;
                            }
                            else if (!WriteTorque(
                                         port_handler,
                                         packet_handler,
                                         test_motor->id,
                                         true))
                            {
                                std::cerr
                                    << "[ERROR] Failed to enable torque."
                                    << std::endl;
                            }
                            else
                            {
                                torque_enabled = true;
                                torque_was_enabled = true;
                                int32_t position_after_torque = 0;
                                if (!ReadPresentPosition(
                                        port_handler,
                                        packet_handler,
                                        test_motor->id,
                                        position_after_torque))
                                {
                                    std::cerr
                                        << "[ERROR] Failed to re-read present "
                                           "position after enabling torque."
                                        << std::endl;
                                }
                                else
                                {
                                    std::cout
                                        << "[INFO] start_position after torque: "
                                        << position_after_torque << std::endl;

                                    if (!IsPositionInRange(
                                            position_after_torque))
                                    {
                                        std::cerr
                                            << "[ERROR] Position after torque "
                                               "is outside [0, 4095]."
                                            << std::endl;
                                    }
                                    else
                                    {
                                        start_position = position_after_torque;
                                        target_position =
                                            start_position + move_tick;
                                        std::cout
                                            << "[INFO] recalculated "
                                               "target_position: "
                                            << target_position << std::endl;

                                        if (!IsPositionInRange(
                                                target_position))
                                        {
                                            std::cerr
                                                << "[ERROR] Recalculated target "
                                                   "is outside [0, 4095]."
                                                << std::endl;
                                        }
                                        else if (!WriteGoalPosition(
                                                     port_handler,
                                                     packet_handler,
                                                     test_motor->id,
                                                     target_position))
                                        {
                                            std::cerr
                                                << "[ERROR] Failed to write "
                                                   "target position."
                                                << std::endl;
                                        }
                                        else
                                        {
                                            std::this_thread::sleep_for(
                                                std::chrono::milliseconds(
                                                    kWaitMs)
                                            );
                                            exit_code = 0;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    catch (const std::exception& exception)
    {
        std::cerr << "[ERROR] Exception: " << exception.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "[ERROR] Unknown exception." << std::endl;
    }

    if (port_open && torque_enabled)
    {
        return_attempted = true;
        if (start_position_available)
        {
            return_succeeded = WriteGoalPosition(
                port_handler,
                packet_handler,
                test_motor->id,
                start_position
            );
            if (return_succeeded)
            {
                std::cout << "[INFO] Return to start commanded." << std::endl;
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(kWaitMs)
                );
            }
            else
            {
                std::cerr
                    << "[ERROR] Failed to command return to start position."
                    << std::endl;
                exit_code = 1;
            }
        }

        torque_disable_attempted = true;
        torque_disable_succeeded = WriteTorque(
            port_handler,
            packet_handler,
            test_motor->id,
            false
        );
        if (torque_disable_succeeded)
        {
            torque_enabled = false;
            std::cout << "[INFO] Torque disabled." << std::endl;
        }
        else
        {
            std::cerr << "[ERROR] Failed to disable torque." << std::endl;
            exit_code = 1;
        }
    }

    if (port_open)
    {
        port_handler->closePort();
        port_open = false;
    }

    if (!id_scan)
    {
        std::cout << "[SUMMARY] TEST_ID=" << test_id_value
                  << " joint_name=" << test_motor->joint_name << std::endl;
        std::cout << "[SUMMARY] start_position="
                  << (start_position_available
                          ? std::to_string(start_position)
                          : "unavailable")
                  << " target_position="
                  << (target_position_available
                          ? std::to_string(target_position)
                          : "not_applicable")
                  << " move_tick=" << move_tick << std::endl;
    }
    std::cout << "[SUMMARY] torque_was_enabled="
              << (torque_was_enabled ? "yes" : "no")
              << std::endl;
    std::cout << "[SUMMARY] return_to_start="
              << (return_attempted
                      ? return_succeeded ? "succeeded" : "failed"
                      : "not_required")
              << std::endl;
    std::cout << "[SUMMARY] torque_disable="
              << (torque_disable_attempted
                      ? torque_disable_succeeded ? "succeeded" : "failed"
                      : "not_required")
              << std::endl;

    if (exit_code == 0)
    {
        std::cout << "[DONE] Motor verification completed safely."
                  << std::endl;
    }
    return exit_code;
}
