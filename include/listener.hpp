/**
 ******************************************************************************
 * @file            listener.hpp
 * @brief           Header for the listener.cpp file
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 09/2024
 * @authors  kamilo.melo@km-robota.com, 09/2024
 ******************************************************************************
 */

#ifndef KMR_MYACTU_LISTENER_HPP
#define KMR_MYACTU_LISTENER_HPP

#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <linux/can.h>

#include "config.hpp"
#include "utils.hpp"
#include "p1_parser.hpp"
#include "p2_parser.hpp"


/**
 * @brief   CAN bus listener, running in its own thread
 */
class Listener {
public:
    Listener(std::vector<Motor*> motors, std::vector<int> ids, int s);
    ~Listener();

    // --------- PID ----------- //
    bool getPID(int id, PIDReport& pidReport);
    bool PID_written_RAM(int id);
    bool PID_written_EEPROM(int id);

    // --------- Acc settings  ----------- //
    bool getAccSettings(int id, ACC_SETTINGS setting, int& acc);
    bool accSettingWritten(int id, ACC_SETTINGS setting);

    // --------- On/off  ----------- //
    bool shutdown_received(int id);
    bool stop_received(int id);
    bool brake_release_received(int id);
    bool brake_lock_received(int id);

    // ---- Status and errors ---- //
    bool getErrorReport(int id, ErrorReport& errorReport);
    bool getPhaseReport(int id, PhaseReport& phaseReport);

    bool getTorque(int id, float& torque);
    bool getSpeed(int id, float& speed);
    bool getAngle(int id, float& angle);
    bool getTemperature(int id, int& temperature);
    bool getFullStatusReport(int id, StatusReport& statusReport);

    // ---- Commands ---- //
    bool torque_command_received(int id);
    bool speedWritten(int id);
    bool hybrid_written(int id);

    // ---- Motor info ---- //
    bool getModel(int id, std::string& model);
    bool getOperatingMode(int id, OperatingMode& mode);
    bool getPowerConsumption(int id, float& power);
    bool getRuntime(int id, float& runtime);
    bool getSoftwareDate(int id, int& date);

    // ---- Other settings ---- //
    bool canFilterWritten(int id);
    bool multiturnResetWritten(int id);
    bool multiturnModeWritten(int id);
    bool activeErrorFbckWritten(int id);

    // ---- Position feedbacks ---- //
    bool getEncoderPosition(int id, int32_t& position);
    bool getRawEncoderPosition(int id, uint32_t& position);
    bool getEncoderZeroOffset(int id, uint32_t& position);
    bool encoderZeroOffsetWritten(int id);

    bool getEncoderFbck_ST(int id, Encoder_ST& encoder_ST);

    bool getPosition_MT(int id, float& angle);
    bool getPosition_ST(int id, float& angle);

    bool positionMT_written(int id);
    bool positionST_written(int id);
    bool positionIncrMT_written(int id);


    // ---- PROTOCOL 2 ---- //
    
    bool defaultCommandType_written(int id);



private:
    // Thread
    bool m_stopThread = 0;
    std::thread m_thread;
    std::mutex m_mutex;

    std::vector<Motor*> m_motors;
    int m_nbrMotors;
    std::vector<int> m_ids;

    // Protocol parsers
    P1Parser* p1Parser = nullptr;
    P2Parser* p2Parser = nullptr;

    int listenerLoop(int s);
    Protocol getProtocol(can_frame frame);
};

#endif