/**
 ****************************************************************************
 * @file        writer.cpp
 * @brief       CAN bus writer
 ****************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 09/2024
 * @authors  kamilo.melo@km-robota.com, 09/2024
 ****************************************************************************
 */

#include <linux/can.h>
#include <iostream>
#include <unistd.h>
#include <cmath>

#include "writer.hpp"

using namespace std;

/**
 * @brief       Create the CAN writer
 * @param[in]   s Socket
 */
Writer::Writer(vector<Motor*> motors, vector<int> ids, int s)
{
    m_s = s;
    m_motors = motors;
	m_nbrMotors = motors.size();
    m_ids = ids;
}

/**
 * @brief	Class destructor
 */
Writer::~Writer()
{

}

/*
 *****************************************************************************
 *                               Motor infos
 ****************************************************************************/

// --------- PID ----------- //

int Writer::requestPID(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x30;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}


int Writer::writePID_RAM(int id, PIDReport pidReport)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x31;
    frame.data[1] = 0x00;
    frame.data[2] = (int8_t) pidReport.Kp_torque;
    frame.data[3] = (int8_t) pidReport.Ki_torque;
    frame.data[4] = (int8_t) pidReport.Kp_speed;
    frame.data[5] = (int8_t) pidReport.Ki_speed;
    frame.data[6] = (int8_t) pidReport.Kp_pos;
    frame.data[7] = (int8_t) pidReport.Ki_pos;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}


int Writer::writePID_EEPROM(int id, PIDReport pidReport)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x32;
    frame.data[1] = 0x00;
    frame.data[2] = (int8_t) pidReport.Kp_torque;
    frame.data[3] = (int8_t) pidReport.Ki_torque;
    frame.data[4] = (int8_t) pidReport.Kp_speed;
    frame.data[5] = (int8_t) pidReport.Ki_speed;
    frame.data[6] = (int8_t) pidReport.Kp_pos;
    frame.data[7] = (int8_t) pidReport.Ki_pos;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}

// --------- Acc settings  ----------- //

int Writer::requestAccSettings(int id, ACC_SETTINGS setting)
{
    int8_t mode = 0;
    switch (setting)
    {
    case POSITION_ACC:
        mode = 0x00;
        break;
    case POSITION_DEC:
        mode = 0x01;
        break;
    case SPEED_ACC:
        mode = 0x02;
        break;
    case SPEED_DEC:
        mode = 0x03;
        break;
    default:
        cout << "Error! Unknown acceleration setting. Exiting" << endl;
        exit(1);
        break;
    }

    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x42;
    frame.data[1] = mode;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}

// Writes to EEPROM. Value between 1.74 and 1047 rad/s²
int Writer::writeAccSettings(int id, ACC_SETTINGS setting, int value)
{
    float convert = rad2deg((float) value);
    value = (int) convert;

    const int min = 100;
    const int max = 60000;
    value = saturate(min, max, value);
    int32_t parameter = (int32_t) value;

    // Parse which setting we're writing to
    int8_t mode = 0;
    switch (setting)
    {
    case POSITION_ACC:
        mode = 0x00;
        break;
    case POSITION_DEC:
        mode = 0x01;
        break;
    case SPEED_ACC:
        mode = 0x02;
        break;
    case SPEED_DEC:
        mode = 0x03;
        break;
    default:
        cout << "Error! Unknown acceleration setting. Exiting" << endl;
        exit(1);
        break;
    }

    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x43;
    frame.data[1] = mode;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = (int8_t) (parameter);
    frame.data[5] = (int8_t) (parameter >> 8);
    frame.data[6] = (int8_t) (parameter >> 16);
    frame.data[7] = (int8_t) (parameter >> 24);

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}

// --------- On/off ----------- //

int Writer::requestShutdown(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x80;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;        
}

int Writer::requestStop(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x81;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;           
}

int Writer::requestReset(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x76;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;           
}

int Writer::requestBrakeRelease(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x77;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;    

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;  
}

int Writer::requestBrakeLock(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x78;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;    

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;  
}


// --------- Status and errors ----------- //

int Writer::requestErrorReport(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x9A;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;  
}

int Writer::requestMotorFbck(int id)
{   
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x9C;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));

    return nbytes;
}   

int Writer::requestPhaseReport(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x9D;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;  
}

// --------- Commands ----------- //

int Writer::writeTorque(int id, float torque)
{
    // Adjust sign to motor's coordinates
    torque = -torque;

    // Convert SI to motor parameter 
    const float unit = 0.01;
    int16_t parameter = 0;

    int16_t absParam = abs(torque/unit);

    if (torque >= 0)
        parameter = absParam;
    else
        parameter = (~absParam) + 1;  // 2's complement

    // Send the data
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;          
    frame.data[0] = 0xA1;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = (int8_t) parameter;
    frame.data[5] = (int8_t) (parameter >> 8);
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}


int Writer::writeSpeed(int id, float speed)
{
    // Adjust units and coordinates
    speed = -rad2deg(speed);

    // Convert SI to motor parameter 
    const float unit = 0.01;
    int32_t parameter = 0;

    int32_t absParam = abs(speed/unit);

    if (speed >= 0)
        parameter = absParam;
    else
        parameter = (~absParam) + 1;

    // Send the data
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;       
    frame.data[0] = 0xA2;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = (int8_t) parameter;
    frame.data[5] = (int8_t) (parameter >> 8);
    frame.data[6] = (int8_t) (parameter >> 16);
    frame.data[7] = (int8_t) (parameter >> 24);

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;   
}

int Writer::writeMotionMode(int id, float pos, float speed, float Kp, float Kd, float Tff)
{
    // Convert the required values
    pos = -pos;
    speed = -speed;
    Tff = -Tff;

    // Saturate the values between accepted intervals
    const float minPos = -12.5, maxPos = 12.5;
    const float minSpeed = -45, maxSpeed = 45;
    const float minKp = 0, maxKp = 500;
    const float minKd = 0, maxKd = 5;
    const float minTff = -24, maxTff = 24; 

    pos = saturate(minPos, maxPos, pos);
    speed = saturate(minSpeed, maxSpeed, speed);
    Kp = saturate(minKp, maxKp, Kp);
    Kd = saturate(minKd, maxKd, Kd);
    Tff = saturate(minTff, maxTff, Tff);

    // Create the packet
    int32_t posParameter = (pos - minPos)*(pow(2,16)-1)/(maxPos - minPos);
    int32_t speedParameter = (speed - minSpeed)*(pow(2,12)-1)/(maxSpeed - minSpeed);
    int32_t KpParameter = (Kp - minKp)*(pow(2,12)-1)/(maxKp - minKp);
    int32_t KdParameter = (Kd - minKd)*(pow(2,12)-1)/(maxKd - minKd);
    int32_t TffParameter = (Tff - minTff)*(pow(2,12)-1)/(maxTff - minTff);

    struct can_frame frame;
    frame.can_id = 0x400 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = (int8_t) (posParameter >> 8); 
    frame.data[1] = (int8_t) posParameter;
    frame.data[2] = (int8_t) (speedParameter >> 4);
    frame.data[3] = ( ((int8_t) speedParameter) << 4) | ( (int8_t) (KpParameter >> 8));
    frame.data[4] = (int8_t) KpParameter;
    frame.data[5] = (int8_t) (KdParameter >> 4);
    frame.data[6] = ( ((int8_t) KdParameter) << 4) | ( (int8_t) (TffParameter >> 8));
    frame.data[7] = (int8_t) TffParameter;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;   
}



// --------- Motor infos ----------- //

int Writer::requestModel(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0xB5;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;   
}

int Writer::requestOperatingMode(int id) 
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x70;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;       
}

int Writer::requestPowerConsumption(int id) 
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x71;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;
}

int Writer::requestRuntime(int id) 
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0xB1;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;
}

int Writer::requestSoftwareDate(int id) 
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0xB2;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;
}

int Writer::enableCANFilter(int id) 
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x20;
    frame.data[1] = 0x02; // CAN filter mode
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 1;    // enable
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;
}

int Writer::disableCANFilter(int id) 
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x20;
    frame.data[1] = 0x02; // CAN filter mode
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0;    // disable
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;
}


// NEED TO RESTART. Resets multiturn, updates 0 and saves to EEPROM
int Writer::requestClearMultiturn(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x20;
    frame.data[1] = 0x01; // Clear multiturn value
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}

int Writer::enableActiveErrorFbck(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x20;
    frame.data[1] = 0x03; // Active error status function
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 1;      // Enable
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}

int Writer::disableActiveErrorFbck(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x20;
    frame.data[1] = 0x03; // Active error status function
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0;      // Disable
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}

int Writer::setMultiturnMode(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x20;
    frame.data[1] = 0x04; // Single vs multi turn function
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 1;      // Multiturn enabled
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;   
}

int Writer::setSingleturnMode(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x20;
    frame.data[1] = 0x04; // Single vs multi turn function
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0;      // Multiturn disabled (= singleturn only)
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;   
}

int Writer::requestEncoderPosition(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x60;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;       
}

int Writer::requestRawEncoderPosition(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x61;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;       
}


int Writer::requestEncoderZeroOffset(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x62;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}

int Writer::writeEncoderZeroOffset(int id, int offset)
{
    if (offset < 0) {
        cout << "Error! Attempt to set a negative encoder zero offset. Exiting" << endl;
        exit(1);
    }

    int32_t offsetParam = (int32_t) offset;  
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x63;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = (int8_t) offsetParam;
    frame.data[5] = (int8_t) (offsetParam >> 8);
    frame.data[6] = (int8_t) (offsetParam >> 16);
    frame.data[7] = (int8_t) (offsetParam >> 24);

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}

int Writer::writeEncoderZeroOffset(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x64;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}

int Writer::requestEncoderFbck_ST(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x90;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}

int Writer::requestPosition_MT(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x92;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}

int Writer::requestPosition_ST(int id)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x94;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}    

int Writer::writePosition_MT(int id, float maxSpeed, float angle)
{   
    if (maxSpeed < 0) {
        cout << "Error? Input max speed is negative in position command. Setting it to 0 instead" << endl;
        maxSpeed = 0;
    }

    // Convert from our custom references
    maxSpeed = rad2deg(maxSpeed);
    angle = -rad2deg(angle);

    // Get parameters
    float unitsSpeed = 1;
    float unitsPosition = 0.01;

    int32_t parameter = 0;
    int32_t absParam = abs(angle/unitsPosition);

    if (angle >= 0)
        parameter = absParam;
    else
        parameter = (~absParam) + 1;

    int16_t maxSpeedParameter = (int16_t) maxSpeed/unitsPosition;
    
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0xA4;
    frame.data[1] = 0x00;
    frame.data[2] = (int8_t) maxSpeedParameter;
    frame.data[3] = (int8_t) (maxSpeedParameter >> 8);  // Speed coded to 0
    frame.data[4] = (int8_t) parameter;
    frame.data[5] = (int8_t) (parameter >> 8);
    frame.data[6] = (int8_t) (parameter >> 16);
    frame.data[7] = (int8_t) (parameter >> 24);

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}


int Writer::writePosition_ST(int id, float maxSpeed, float angle)
{   
    // Check the validity of input data
    if (maxSpeed < 0) {
        cout << "Error? Input max speed is negative in position command. Setting it to 0 instead" << endl;
        maxSpeed = 0;
    }
    if (angle > M_PI) {
        cout << "Error! Input angle > pi in single turn command. Setting it to PI instead" << endl;
        angle = M_PI;
    }
    else if (angle < -M_PI) {
        cout << "Error! Input angle < -pi in single turn command. Setting it to -PI instead" << endl;
        angle = -M_PI;
    }

    // Convert from our custom references
    maxSpeed = rad2deg(maxSpeed);
    angle = -rad2deg(angle);

    // DEBUG
    angle = 360; // deg
    maxSpeed = 500; // dps

    // Get parameters
    float unitsSpeed = 1;
    float unitsPosition = 0.01;

    int16_t parameter = abs(angle/unitsPosition);
    int8_t spinDirection = 0;

    if (angle >= 0)
        spinDirection = 0;
    else
        spinDirection = 1;

    int16_t maxSpeedParameter = (int16_t) maxSpeed/unitsSpeed;
    
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0xA6;
    frame.data[1] = spinDirection;
    frame.data[2] = (int8_t) maxSpeedParameter;
    frame.data[3] = (int8_t) (maxSpeedParameter >> 8); 
    frame.data[4] = (int8_t) parameter;
    frame.data[5] = (int8_t) (parameter >> 8);
    frame.data[6] = 0;
    frame.data[7] = 0;

    cout << "data 1:" << (int)frame.data[1]<< endl;
    cout << "data 2:" << (int)frame.data[2]<< endl;
    cout << "data 3:" << (int)frame.data[3]<< endl;
    cout << "data 4:" << (int)frame.data[4]<< endl;
    cout << "data 5:" << (int)frame.data[5]<< endl;
    cout << "data 6:" << (int)frame.data[6]<< endl;
    cout << "data 7:" << (int)frame.data[7]<< endl;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}
