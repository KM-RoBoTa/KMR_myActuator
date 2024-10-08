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
#include <stdint.h>
#include <vector>
#include <algorithm>

#define FRAME_LENGTH        8
#define RESPONSE_TIMEOUT    30*1000 // us

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

struct Encoder_ST {
    uint16_t encoderPosition;
    uint16_t encoderRawPosition;
    uint16_t encoderZeroOffset;
};

enum OperatingMode {
    TORQUE_LOOP, SPEED_LOOP, POSITION_LOOP
};

enum Protocol {
    PROTOCOL_1 = 1, PROTOCOL_2 = 2, UNDEF_PTC
};

enum Model {
    X6_7 = 0, 
    X6_40 = 1,
    X8_25 = 2, 
    X6_8 = 3,
    X8_20 = 4,
    X8_60 = 5,
    X10_40 = 6,
    X10_100 = 7,
    X4_24 = 8, 
    X8_90 = 9, 
    X12_150 = 10,
    X15_400 = 11,
    UNDEF_MODEL
};

enum ACC_SETTINGS {
    POSITION_ACC, POSITION_DEC, SPEED_ACC, SPEED_DEC
};

/**
 * @brief   Structure saving the info of a data field
 */
struct Motor {
    int id;
    Protocol protocol = UNDEF_PTC;
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

    int32_t encoderPosition = 0;
    uint32_t encoderRawPosition = 0;
    uint32_t encoderZeroOffset = 0;

    // Settings for custom single turn
    float maxAngle =  M_PI;
    float minAngle = -M_PI; 
    int k = 0;              // factor for the 2kpi reference offset
    float refOffset = 0;
    bool customST = 0;      // Flag if custom single turn mode is enabled
    bool limitAngles = 0;   // Flag if angle limits are active

    // ST???
    uint16_t encoderPosition_ST = 0;
    uint16_t encoderRawPosition_ST = 0;
    uint16_t encoderZeroOffset_ST = 0;

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
    bool fw_positionIncr_MT = 0;


    // Constructor
    Motor(int id, Model model)
    {
        this->id = id;
        this->model = model;

        // Save protocol corresponding to the model
        if (model == UNDEF_MODEL) {
            std::cout << "Model for motor " << id << " is undefined!" << std::endl;
            std::cout << std::endl;
        }

        std::vector<Model> p1_models = {X6_7, X6_40, X8_25, X6_8, X8_20, X8_60, X10_40, X10_100};
        std::vector<Model> p2_models = {X4_24, X8_90, X12_150, X15_400};

        int cnt = std::count(p1_models.begin(), p1_models.end(), model);
        if (cnt > 0) 
            protocol = PROTOCOL_1;
        else {
            cnt = std::count(p2_models.begin(), p2_models.end(), model);

            if (cnt > 0)
                protocol = PROTOCOL_2;
            else {
                std::cout << "Could not identify the protocol of motor " << id << ". Exiting" << std::endl;
                exit(1);
            }
        }
    }
};





#endif
