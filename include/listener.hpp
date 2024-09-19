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


/**
 * @brief   CAN bus listener, running in its own thread
 */
class Listener {
public:
    Listener(std::vector<Motor*> motors, std::vector<int> ids, int s);
    ~Listener();

    void getModel(int id, bool& hasResponded, char model[]);

    // --------- PID ----------- //
    bool getPID(int id, PacketPID& packetPID);
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

    // Status fbck
    float getTorque(int id);
    float getSpeed(int id);
    float getAngle(int id);
    float getTemperature(int id);
    
    bool speedWritten(int id);



private:
    // Thread
    bool m_stopThread = 0;
    std::thread m_thread;
    std::mutex m_mutex;

    std::vector<Motor*> m_motors;
    int m_nbrMotors;
    std::vector<int> m_ids;

    int listenerLoop(int s);

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


    void readModel(can_frame frame);

    // PARSE SPEED
    void parseTorqueCommand(can_frame frame);
    void parseSpeedCommand(can_frame frame);

    // Status fbck
    void parseMotorFbck(can_frame frame);
};

#endif