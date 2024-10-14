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

#ifndef KMR_MYACTU_P1_PARSER_HPP
#define KMR_MYACTU_P1_PARSER_HPP

#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <linux/can.h>

#include "config.hpp"
#include "utils.hpp"


/**
 * @brief   Parser for protocol 1
 */
class P1Parser {
public:
    P1Parser(std::vector<Motor*> motors, std::vector<int> ids, std::mutex* mutex);
    //~P1Parser();

    void parseFrame(can_frame frame); 

private:
    std::mutex* m_mutex;
    std::vector<Motor*> m_motors;
    int m_nbrMotors;
    std::vector<int> m_ids;

    // --------- PID ----------- //
    void parsePIDFbck(can_frame frame);
    void parsePID_RAM_write(can_frame frame);
    void parsePID_EEPROM_write(can_frame frame);

    // --------- Acc settings  ----------- //
    void parseAccSettingsFbck(can_frame frame);
    void parseAccSettings_write(can_frame frame);

    // --------- On/off  ----------- //
    void parseShutdown(can_frame frame);
    void parseStop(can_frame frame);
    void parseBrakeRelease(can_frame frame);
    void parseBrakeLock(can_frame frame);

    // ---- Status and errors ---- //
    void parseErrorReport(can_frame frame);
    void parseMotorFbck(can_frame frame);
    void parsePhaseReport(can_frame frame);

    // ---- Commands ---- //
    void parseTorqueCommand(can_frame frame);
    void parseSpeedCommand(can_frame frame);
    void parseHybridCommand(can_frame frame);

    // ---- Motor info ---- //
    //void parseModel(can_frame frame);
    void parseOperatingMode(can_frame frame);
    void parsePowerConsumption(can_frame frame);
    void parseRuntime(can_frame frame);
    void parseSoftwareDate(can_frame frame);

    // ---- Other settings ---- //
    void parseCompoundFct(can_frame frame);
    void parseCANFilter(can_frame frame);
    void parseMultiturnReset(can_frame frame);
    void parseMultiturnMode(can_frame frame);
    void parseActiveError(can_frame frame);

    // ---- Position feedbacks ---- //
    void parseEncoderFbck(can_frame frame);
    void parseRawEncoderFbck(can_frame frame);
    void parseEncoderZeroOffsetRead(can_frame frame);
    void parseEncoderZeroOffsetWrite(can_frame frame);

    void parseEncoderFbck_ST(can_frame frame);

    void parsePositionFbck_MT(can_frame frame);
    void parsePositionFbck_ST(can_frame frame);

    void parsePositionCommand_MT(can_frame frame);
    void parsePositionCommand_ST(can_frame frame);
    void parsePositionIncrCommand_MT(can_frame frame);
};

#endif