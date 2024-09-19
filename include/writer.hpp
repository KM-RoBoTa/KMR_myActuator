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
    int writePID_RAM(int id, PacketPID packetPID);      // 0x31
    int writePID_EEPROM(int id, PacketPID packetPID);   // 0x32

    // Acceleration settings
    int requestAccSettings(int id, ACC_SETTINGS setting);           // 0x42
    int writeAccSettings(int id, ACC_SETTINGS setting, int value);  // 0x43

    // Motor infos
    int requestModel(int id);
    int writeTorque(int id, float torque);
    int writeSpeed(int id, float speed);
    int requestMotorFbck(int id);


private:
    int m_s; // Socket
    std::vector<int> m_ids;

    std::vector<Motor*> m_motors;
    int m_nbrMotors;

};


#endif