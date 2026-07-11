#include "dynamixel.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

int main()
{
    const uint8_t TEST_ID = 17;        // RKN, 오른무릎 후보
    const int32_t MOVE_TICK = 10;      // 아주 작게 이동
    const int WAIT_MS = 1000;

    auto portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    auto packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    if (!portHandler->openPort()) {
        std::cerr << "[ERROR] Failed to open port: " << DEVICE_NAME << std::endl;
        return 1;
    }

    if (!portHandler->setBaudRate(BAUDRATE)) {
        std::cerr << "[ERROR] Failed to set baudrate: " << BAUDRATE << std::endl;
        portHandler->closePort();
        return 1;
    }

    std::cout << "[INFO] Port opened. Testing motor ID " << int(TEST_ID) << std::endl;

    uint32_t present_position = 0;

    dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        TEST_ID,
        DxlReg_PresentPosition,
        &present_position,
        &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "[ERROR] Failed to read present position." << std::endl;
        portHandler->closePort();
        return 1;
    }

    int32_t start_position = static_cast<int32_t>(present_position);
    int32_t target_position = start_position + MOVE_TICK;

    std::cout << "[INFO] Present position: " << start_position << std::endl;
    std::cout << "[INFO] Target position : " << target_position << std::endl;
    std::cout << "[WARN] Motor will move only +" << MOVE_TICK << " tick." << std::endl;
    std::cout << "Press ENTER to continue, or Ctrl+C to cancel." << std::endl;
    std::cin.get();

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        TEST_ID,
        DxlReg_TorqueEnable,
        1,
        &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "[ERROR] Failed to enable torque." << std::endl;
        portHandler->closePort();
        return 1;
    }

    packetHandler->write4ByteTxRx(
        portHandler,
        TEST_ID,
        DxlReg_GoalPosition,
        target_position,
        &dxl_error
    );

    std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_MS));

    packetHandler->write4ByteTxRx(
        portHandler,
        TEST_ID,
        DxlReg_GoalPosition,
        start_position,
        &dxl_error
    );

    std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_MS));

    packetHandler->write1ByteTxRx(
        portHandler,
        TEST_ID,
        DxlReg_TorqueEnable,
        0,
        &dxl_error
    );

    portHandler->closePort();

    std::cout << "[DONE] Returned to start position and torque off." << std::endl;
    return 0;
}
