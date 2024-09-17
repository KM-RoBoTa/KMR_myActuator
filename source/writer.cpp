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