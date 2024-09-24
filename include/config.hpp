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

#include <string>

#define FRAME_LENGTH    8
#define RESPONSE_TIMEOUT 30*1000 // us

#define ERR_MOTOR_STALL     0x0002
#define ERR_UNDERVOLTAGE    0x0004
#define ERR_OVERVOLTAGE     0x0008
#define ERR_OVERCURRENT     0x0010
#define ERR_POWER_OVERRUN   0x0040
#define ERR_CALIB_PARAM     0x0080
#define ERR_SPEEDING        0x0100
#define ERR_OVERHEATING     0x1000
#define ERR_ENCODER_CALIB   0x2000


struct ErrorReport {
    int temperature = 0;
    int brakeReleased = 0;
    float voltage = 0;
    bool err_motorStall = 0;
    bool err_undervoltage = 0;
    bool err_overvoltage = 0;
    bool err_overcurrent = 0;
    bool err_powerOverrun = 0;
    bool err_calibParameter = 0;
    bool err_speeding = 0;
    bool err_overheating = 0;
    bool err_encoderCalib = 0;
};

struct PhaseReport {
    int temperature = 0;
    float currentPhaseA = 0;
    float currentPhaseB = 0;
    float currentPhaseC = 0;
};

struct PIDReport {
    int Kp_torque = 100;
    int Ki_torque = 100;
    int Kp_speed = 100;
    int Ki_speed = 50;
    int Kp_pos = 100;
    int Ki_pos = 50;
};

struct StatusReport {
    int temperature;
    float torque;
    float speed; 
    float angle;
};


enum OperatingMode {
    TORQUE_LOOP, SPEED_LOOP, POSITION_LOOP
};

struct Encoder_ST {
    int encoderPosition;
    int encoderRawPosition;
    int encoderZeroOffset;
};


/**
 * @brief   Structure saving the info of a data field
 */
struct Motor {
    int id;
    //char model[FRAME_LENGTH-1];
    std::string model;
    int temperature;
    float torque;
    float speed; 
    float angle;

    float angle_posFbck_MT; // SAME AS ABOVE??
    float angle_posFbck_ST; // SAME AS ABOVE??

    PIDReport pidReport;

    int positionAcc = 174; // rad/s², equivalent to 10000 deg/s²
    int positionDec = 174;
    int speedAcc = 174;
    int speedDec = 174;

    ErrorReport errorReport;
    PhaseReport phaseReport;

    OperatingMode operatingMode;
    float power; // W
    float runtime; // s
    int softwareDate;

    int encoderPosition = 0;
    int encoderRawPosition = 0;
    int encoderZeroOffset = 0;

    // ST???
    int encoderPosition_ST = 0;
    int encoderRawPosition_ST = 0;
    int encoderZeroOffset_ST = 0;

    // Update flags
    bool f_model = 0;
    bool fr_temperature = 0;
    bool fr_torque = 0;
    bool fr_speed = 0;
    bool fr_angle = 0;
    bool fr_fullFbck = 0;

    bool fr_PID = 0;        // flag read PID
    bool fw_PID_RAM = 0;
    bool fw_PID_EEPROM = 0;

    bool f_shutdown = 0;
    bool f_stop = 0;
    bool f_brake = 0;

    bool fr_posAcc = 0;
    bool fr_posDec = 0;
    bool fr_speedAcc = 0;
    bool fr_speedDec = 0;
    bool fw_posAcc = 0;
    bool fw_posDec = 0;
    bool fw_speedAcc = 0;
    bool fw_speedDec = 0; 

    bool f_errorReport = 0;
    //
    bool f_phaseReport = 0;

    bool fw_torque = 0;
    bool fw_speed = 0;
    bool fw_motion = 0;

    bool fr_mode = 0;
    bool fr_power = 0;
    bool fr_runtime = 0;
    bool fr_softwareDate;

    bool fw_CANFilter = 0;
    bool fw_multiturnReset = 0;
    bool fw_multiturnMode = 0;
    bool fw_activeErrorFbck = 0;

    bool fr_encoder = 0;
    bool fr_encoderRaw = 0;
    bool fr_encoderZeroOffset = 0;
    bool fw_encoderZeroOffset = 0;

    // ST??
    bool fr_encoder_ST = 0;
    bool fr_encoderRaw_ST = 0;
    bool fr_encoderZeroOffset_ST = 0; 

    bool fr_position_MT = 0; // SAME AS OTHER???
    bool fr_position_ST = 0;
    bool fw_position_MT = 0; // SAME AS OTHER???
    bool fw_position_ST = 0; 



    // Constructor
    Motor(int id)
    {
        this->id = id;
    }
};

enum ACC_SETTINGS {
    POSITION_ACC, POSITION_DEC, SPEED_ACC, SPEED_DEC
};



#endif
