/**
 ****************************************************************************
 * @file        p2_writer.cpp
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

#include "p2_writer.hpp"

using namespace std;

/**
 * @brief       Create the CAN writer
 * @param[in]   s Socket
 */
P2Writer::P2Writer(vector<Motor*> motors, vector<int> ids, int s)
{
    m_s = s;
    m_motors = motors;
	m_nbrMotors = motors.size();
    m_ids = ids;
}

/**
 * @brief	Class destructor
 */
P2Writer::~P2Writer()
{

}

/*
 *****************************************************************************
 *                              
 ****************************************************************************/

int P2Writer::setDefaultCommandType(int id)
{
    uint16_t uid = (uint16_t) id;

    struct can_frame frame;
    frame.can_id = 0x7FF;
    frame.len = 4;
    frame.data[0] = (uint8_t) (uid >> 8);
    frame.data[1] = (uint8_t) uid;
    frame.data[2] = 0x00; // Command message
    frame.data[3] = 0x02; // "Q&A mode"

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;    
}


int P2Writer::setMultiturnMode(int id)
{
    uint16_t uid = (uint16_t) id;

    struct can_frame frame;
    frame.can_id = 0x7FF;
    frame.len = 4;
    frame.data[0] = (uint8_t) (uid >> 8);
    frame.data[1] = (uint8_t) uid;
    frame.data[2] = 0x00; // Command message
    frame.data[3] = 0x03; // Set zero (also resets multiturn counter)

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;       
}


int P2Writer::writeHybrid(int id, float pos, float speed, float Kp, float Kd, float Tff)
{
    // Saturate the values between accepted intervals
    const float minPos = -12.5, maxPos = 12.5;
    const float minSpeed = -18, maxSpeed = 18;
    const float minKp = 0, maxKp = 500;
    const float minKd = 0, maxKd = 5;
    const float minTff = -90, maxTff = 90; 

    pos = saturate(minPos, maxPos, pos);
    speed = saturate(minSpeed, maxSpeed, speed);
    Kp = saturate(minKp, maxKp, Kp);
    Kd = saturate(minKd, maxKd, Kd);
    Tff = saturate(minTff, maxTff, Tff);

    // Create packet
    /*struct can_frame frame;
    frame.can_id = id;
    frame.len = 8;
    frame.data[0] = (uint8_t) (uid >> 8);
    frame.data[1] = (uint8_t) uid;
    frame.data[2] = 0x00; // Command message
    frame.data[3] = 0x03; // Set zero (also resets multiturn counter)

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;         */  
}


int P2Writer::writeSpeed(int id, float speed)
{
    // Min/max saturation values for speed? Message return type?
    // Threshold current value?
}

int P2Writer::writeTorque(int id, float torque)
{
    // Saturate the values between accepted intervals
    const float minTorque = -327.68, maxTorque = 327.67;
    saturate(minTorque, maxTorque, torque);

    float unitsTorque = 0.01;
    int16_t paramTorque = torque/unitsTorque;

    // Create packet
    /*struct can_frame frame;
    frame.can_id = id;
    frame.len = 8;
    frame.data[0] = (uint8_t) (uid >> 8);
    frame.data[1] = (uint8_t) uid;
    frame.data[2] = 0x00; // Command message
    frame.data[3] = 0x03; // Set zero (also resets multiturn counter)

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;         */  
}

