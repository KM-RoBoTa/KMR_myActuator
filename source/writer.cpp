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


int Writer::writePID_RAM(int id, PacketPID packetPID)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x31;
    frame.data[1] = 0x00;
    frame.data[2] = (int8_t) packetPID.Kp_torque;
    frame.data[3] = (int8_t) packetPID.Ki_torque;
    frame.data[4] = (int8_t) packetPID.Kp_speed;
    frame.data[5] = (int8_t) packetPID.Ki_speed;
    frame.data[6] = (int8_t) packetPID.Kp_pos;
    frame.data[7] = (int8_t) packetPID.Ki_pos;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}


int Writer::writePID_EEPROM(int id, PacketPID packetPID)
{
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x32;
    frame.data[1] = 0x00;
    frame.data[2] = (int8_t) packetPID.Kp_torque;
    frame.data[3] = (int8_t) packetPID.Ki_torque;
    frame.data[4] = (int8_t) packetPID.Kp_speed;
    frame.data[5] = (int8_t) packetPID.Ki_speed;
    frame.data[6] = (int8_t) packetPID.Kp_pos;
    frame.data[7] = (int8_t) packetPID.Ki_pos;

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
    int nbytes = write(m_s, &frame, sizeof(can_frame));

    if (nbytes >= 0)
        cout << "Get model sent" << endl;
    else
        cout << "Problem with getting model" << endl;

    //return nbytes;   
    return 1;
}

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

int Writer::requestMotorFbck(int id)
{   
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = FRAME_LENGTH;          // 8 bytes
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

