// Build: g++ -std=c++17 single_motor_test.cpp -o single_motor_test -L/usr/local/lib -ldxl_x64_cpp -pthread

#include "dynamixel.hpp"

#include <chrono>
#include <cmath>
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

class PortCloseGuard
{
public:
    PortCloseGuard(dynamixel::PortHandler* port_handler, bool& port_open)
        : port_handler_(port_handler), port_open_(port_open)
    {
    }

    ~PortCloseGuard()
    {
        if (port_open_) {
            port_handler_->closePort();
        }
    }

private:
    dynamixel::PortHandler* port_handler_;
    bool& port_open_;
};

int main()
{
    constexpr uint8_t test_id = static_cast<uint8_t>(TEST_ID);  // 17: RKN, 18: LKN
    constexpr int32_t move_tick = static_cast<int32_t>(MOVE_TICK);
    const int WAIT_MS = 1000;
    const int32_t MIN_POSITION = 0;
    const int32_t MAX_POSITION = 4095;

    if (TEST_ID != 17 && TEST_ID != 18) {
        std::cerr << "[ERROR] TEST_ID must be 17 (RKN) or 18 (LKN)." << std::endl;
        return 1;
    }

    if (std::abs(static_cast<int64_t>(MOVE_TICK)) > 30) {
        std::cerr << "[ERROR] |MOVE_TICK| must not exceed 30." << std::endl;
        return 1;
    }

    auto portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    auto packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    bool port_open = false;
    PortCloseGuard port_close_guard(portHandler, port_open);
    bool torque_enable_attempted = false;
    bool start_position_available = false;
    int32_t start_position = 0;
    int exit_code = 1;

    try {
        if (!portHandler->openPort()) {
            std::cerr << "[ERROR] Failed to open port: " << DEVICE_NAME << std::endl;
        } else {
            port_open = true;

            if (!portHandler->setBaudRate(BAUDRATE)) {
                std::cerr << "[ERROR] Failed to set baudrate: " << BAUDRATE << std::endl;
            } else {
                uint8_t dxl_error = 0;
                uint32_t present_position = 0;
                int dxl_comm_result = packetHandler->read4ByteTxRx(
                    portHandler,
                    test_id,
                    DxlReg_PresentPosition,
                    &present_position,
                    &dxl_error
                );

                if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
                    std::cerr << "[ERROR] Failed to read present position." << std::endl;
                } else {
                    start_position = static_cast<int32_t>(present_position);
                    start_position_available = true;
                    int32_t target_position = start_position + move_tick;

                    if (target_position < MIN_POSITION || target_position > MAX_POSITION) {
                        std::cerr << "[ERROR] Target position is outside the valid range [0, 4095]."
                                  << std::endl;
                    } else {
                        std::cout << "[INFO] TEST_ID         : " << static_cast<int>(test_id) << std::endl;
                        std::cout << "[INFO] start_position  : " << start_position << std::endl;
                        std::cout << "[INFO] target_position : " << target_position << std::endl;
                        std::cout << "[INFO] MOVE_TICK       : " << move_tick << std::endl;

                        std::string confirmation;
                        std::cout << "Type YES to continue" << std::endl;
                        std::getline(std::cin, confirmation);

                        if (confirmation != "YES") {
                            std::cerr << "[ABORT] First confirmation failed." << std::endl;
                        } else {
                            std::cout << "Type MOTOR_ON to enable torque" << std::endl;
                            std::getline(std::cin, confirmation);

                            if (confirmation != "MOTOR_ON") {
                                std::cerr << "[ABORT] Second confirmation failed." << std::endl;
                            } else {
                                dxl_error = 0;
                                torque_enable_attempted = true;
                                dxl_comm_result = packetHandler->write1ByteTxRx(
                                    portHandler,
                                    test_id,
                                    DxlReg_TorqueEnable,
                                    1,
                                    &dxl_error
                                );

                                if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
                                    std::cerr << "[ERROR] Failed to enable torque." << std::endl;
                                } else {
                                    dxl_error = 0;
                                    present_position = 0;
                                    start_position_available = false;
                                    dxl_comm_result = packetHandler->read4ByteTxRx(
                                        portHandler,
                                        test_id,
                                        DxlReg_PresentPosition,
                                        &present_position,
                                        &dxl_error
                                    );

                                    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
                                        std::cerr << "[ERROR] Failed to re-read present position "
                                                     "after enabling torque." << std::endl;
                                    } else {
                                        start_position = static_cast<int32_t>(present_position);
                                        start_position_available = true;
                                        target_position = start_position + move_tick;

                                        std::cout << "[INFO] start_position after torque enable  : "
                                                  << start_position << std::endl;
                                        std::cout << "[INFO] target_position after recalculation: "
                                                  << target_position << std::endl;

                                        if (target_position < MIN_POSITION
                                            || target_position > MAX_POSITION) {
                                            std::cerr << "[ERROR] Recalculated target position is "
                                                         "outside the valid range [0, 4095]."
                                                      << std::endl;
                                        } else {
                                            dxl_error = 0;
                                            dxl_comm_result = packetHandler->write4ByteTxRx(
                                                portHandler,
                                                test_id,
                                                DxlReg_GoalPosition,
                                                static_cast<uint32_t>(target_position),
                                                &dxl_error
                                            );

                                            if (dxl_comm_result != COMM_SUCCESS
                                                || dxl_error != 0) {
                                                std::cerr << "[ERROR] Failed to write target position."
                                                          << std::endl;
                                            } else {
                                                std::this_thread::sleep_for(
                                                    std::chrono::milliseconds(WAIT_MS)
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
    } catch (const std::exception& exception) {
        std::cerr << "[ERROR] Exception: " << exception.what() << std::endl;
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception." << std::endl;
    }

    if (port_open && torque_enable_attempted) {
        if (start_position_available) {
            uint8_t dxl_error = 0;
            const int result = packetHandler->write4ByteTxRx(
                portHandler,
                test_id,
                DxlReg_GoalPosition,
                static_cast<uint32_t>(start_position),
                &dxl_error
            );

            if (result != COMM_SUCCESS || dxl_error != 0) {
                std::cerr << "[ERROR] Failed to command return to start position." << std::endl;
                exit_code = 1;
            } else {
                std::cout << "[INFO] Return to start position commanded." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_MS));
            }
        }

        uint8_t dxl_error = 0;
        const int result = packetHandler->write1ByteTxRx(
            portHandler,
            test_id,
            DxlReg_TorqueEnable,
            0,
            &dxl_error
        );

        if (result != COMM_SUCCESS || dxl_error != 0) {
            std::cerr << "[ERROR] Failed to disable torque." << std::endl;
            exit_code = 1;
        } else {
            std::cout << "[INFO] Torque disabled." << std::endl;
        }
    }

    if (port_open) {
        portHandler->closePort();
        port_open = false;
    }

    if (exit_code == 0) {
        std::cout << "[DONE] Single-motor test completed safely." << std::endl;
    }

    return exit_code;
}
