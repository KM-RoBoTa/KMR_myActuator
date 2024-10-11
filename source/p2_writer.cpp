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
