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

#ifndef KMR_MYACTU_CONFIG_HPP
#define KMR_MYACTU_CONFIG_HPP

#define FRAME_LENGTH    8
#define RESPONSE_TIMEOUT 30*1000 // us

/**
 * @brief   Structure saving the info of a data field
 */
struct Motor {
    int id;
    char model[FRAME_LENGTH-1];
    int temperature;
    float torque;
    float speed; 
    float angle;

    int Kp_torque = 100;
    int Ki_torque = 100;
    int Kp_speed = 100;
    int Ki_speed = 50;
    int Kp_pos = 100;
    int Ki_pos = 50;

    int positionAcc = 174; // rad/s², equivalent to 10000 deg/s²
    int positionDec = 174;
    int speedAcc = 174;
    int speedDec = 174;

    // Update flags
    bool f_model = 0;
    bool f_temperature = 0;
    bool f_torque = 0;
    bool f_speed = 0;
    bool f_angle = 0;

    bool fr_PID = 0;        // flag read PID
    bool fw_PID_RAM = 0;
    bool fw_PID_EEPROM = 0;

    bool f_shutdown = 0;
    bool f_stop = 0;
    bool f_brake = 0;

    bool fw_speed = 0;
    bool fr_posAcc = 0;
    bool fr_posDec = 0;
    bool fr_speedAcc = 0;
    bool fr_speedDec = 0;
    bool fw_posAcc = 0;
    bool fw_posDec = 0;
    bool fw_speedAcc = 0;
    bool fw_speedDec = 0; 

    // Constructor
    Motor(int id)
    {
        this->id = id;
    }
};

struct PacketPID {
    int Kp_torque = 100;
    int Ki_torque = 100;
    int Kp_speed = 100;
    int Ki_speed = 50;
    int Kp_pos = 100;
    int Ki_pos = 50;
};

enum ACC_SETTINGS {
    POSITION_ACC, POSITION_DEC, SPEED_ACC, SPEED_DEC
};

#endif