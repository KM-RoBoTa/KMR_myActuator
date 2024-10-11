/**
 ****************************************************************************
 * @file        p2_parser.cpp
 * @brief       CAN bus parser for protocol 2
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

#include "p2_parser.hpp"

using namespace std;

extern vector<Model> p1_models;
extern vector<Model> p2_models;

/**
 * @brief       
 */
P2Parser::P2Parser(vector<Motor*> motors, vector<int> ids, mutex* mutex)
{
    m_mutex = mutex;
	m_motors = motors;
	m_nbrMotors = motors.size();
	m_ids = ids;
}

void P2Parser::parseFrame(can_frame frame)
{
    int frame_id = frame.can_id;
    uint8_t command = frame.data[3];

    if (frame_id == 0x7FF) {
        switch (command)
        {
        case 0x00:  cout << "A command failed" << endl; break;
        case 0x02:	parseDefaultCommandType(frame); break; 
        case 0x03:  parseMultiturnMode(frame); break;

        default:
            cout << "Unknown function in protocol 2" << endl; 
            break;
        }
    }
    else {

    }


}

/*
 *****************************************************************************
 *                               Motor infos
 ****************************************************************************/

void P2Parser::parseDefaultCommandType(can_frame frame)
{
    // Confirm sender (0 = controller, 1 = motor)
    int sender = (int)frame.data[2];
    if (sender == 0x00)
        cout << "Caught the command to set the default command type..." << endl;
    else if (sender == 0x01) {
        // Extract the motor ID from the received frame
        int id = (int)frame.data[0]<<8 + (int)frame.data[1];

        // Get the vector's index
        int idx = getIndex(m_ids, id);	

        // Set up confirmation flag
        scoped_lock lock(*m_mutex);
        m_motors[idx]->fw_commandType = 1;
    }
    else 
        cout << "Error! Unknown packet parsed in default command type, protocol 2" << endl;
}

void P2Parser::parseMultiturnMode(can_frame frame)
{
    // Confirm sender (0 = controller, 1 = motor)
    int sender = (int)frame.data[2];
    if (sender == 0x00)
        cout << "Caught the command ..." << endl;
    else if (sender == 0x01) {
        // Extract the motor ID from the received frame
        int id = (int)frame.data[0]<<8 + (int)frame.data[1];

        // Get the vector's index
        int idx = getIndex(m_ids, id);	

        // Set up confirmation flag
        scoped_lock lock(*m_mutex);
        m_motors[idx]->fw_multiturnMode = 1;
    }
    else 
        cout << "Error! Unknown packet parsed in default command type, protocol 2" << endl;    
}






