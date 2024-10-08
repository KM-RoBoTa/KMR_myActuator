/**
 ******************************************************************************
 * @file            writer.hpp
 * @brief           Header for the writer.cpp file
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 09/2024
 * @authors  kamilo.melo@km-robota.com, 09/2024
 ******************************************************************************
 */

#ifndef KMR_MYACTU_WRITER_HPP
#define KMR_MYACTU_WRITER_HPP

#include <vector>

#include "config.hpp"
#include "utils.hpp"

/**
 * @brief   CAN bus writer
 */
class Writer {
public:
    Writer(std::vector<Motor*> motors, std::vector<int> ids, int s);
    ~Writer();

    // PID settings
    int requestPID(int id);                             // 0x30
    int writePID_RAM(int id, PIDReport pidReport);      // 0x31
    int writePID_EEPROM(int id, PIDReport pidReport);   // 0x32

    // Acceleration settings
    int requestAccSettings(int id, ACC_SETTINGS setting);           // 0x42
    int writeAccSettings(int id, ACC_SETTINGS setting, int value);  // 0x43

    // On/off
    int requestShutdown(int id);        // 0x80
    int requestStop(int id);            // 0x81   
    int requestReset(int id);           // 0x76     
    int requestBrakeRelease(int id);    // 0x77
    int requestBrakeLock(int id);       // 0x78

    // Status and errors
    int requestErrorReport(int id);     // 0x9A
    int requestMotorFbck(int id);       // 0x9C
    int requestPhaseReport(int id);     // 0x9D

    // Commands
    int writeTorque(int id, float torque);  // 0xA1
    int writeSpeed(int id, float speed);    // 0xA2
    int writeMotionMode(int id, float pos, float speed, float Kp, float Kd, float Tff); // 0x400
    // !!!!!!!!!!!!!!!!! Very specific case !!!!!!!!!!!!!!¨

    // Motor infos
    int requestModel(int id);               // 0xB5
    int requestOperatingMode(int id);       // 0x70
    int requestPowerConsumption(int id);    // 0x71
    int requestRuntime(int id);             // 0xB1
    int requestSoftwareDate(int id);        // 0xB2
    // !!!!!!!!!!!!!!!!! Weird numbers !!!!!!!!!!!!!!¨

    // Others settings
    int enableCANFilter(int id);            // 0x20
    int disableCANFilter(int id);            // 0x20

    int requestClearMultiturn(int id);      // 0x20
    int setMultiturnMode(int id);           // 0x20
    int setSingleturnMode(int id);          // 0x20

    int enableActiveErrorFbck(int id);  // 0x20
    int disableActiveErrorFbck(int id);  // 0x20

    // Position feedbacks
    int requestEncoderPosition(int id);     // 0x60 (includes 0 offset)
    int requestRawEncoderPosition(int id);     // 0x61 (without 0 offset)
    int requestEncoderZeroOffset(int id);  // 0x62
    int writeEncoderZeroOffset(int id, uint32_t offset);     // 0x63
    int writeEncoderZeroOffset(int id);     // 0x64

    // ST
    int requestEncoderFbck_ST(int id); // 0x90

    // Angles
    int requestPosition_MT(int id); // 0x92
    int requestPosition_ST(int id); // 0x94

    int writePosition_MT(int id, float maxSpeed, float angle);   // 0xA4
    int writePosition_ST(int id, float maxSpeed, float angle);   // 0xA6
    int writePositionIncrement_MT(int id, float maxSpeed, float increment); // 0xA8

    











private:
    int m_s; // Socket
    std::vector<int> m_ids;

    std::vector<Motor*> m_motors;
    int m_nbrMotors;

};


#endif