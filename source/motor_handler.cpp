/**
 ******************************************************************************
 * @file        motor_handler.cpp
 * @brief       Handles the communication with MyActuator Motors
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 09/2024
 * @authors  kamilo.melo@km-robota.com, 09/2024
 ******************************************************************************
 */

// Standard libraries
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <cmath>

#include <sys/socket.h>
#include <linux/can.h>
#include <sys/ioctl.h>
#include <cstring>
#include <net/if.h> // network devices
#include <sstream> // Convert dec-hex

#include "motor_handler.hpp"

using namespace std;

MotorHandler::MotorHandler(vector<int> ids, const char* can_bus)
{
    m_ids = ids;

    for (int i=0; i<ids.size(); i++) {
        Motor* motor = new Motor(ids[i]);
        m_motors.push_back(motor);
    }

    // Open a socket to be able to communicate over a CAN network
    int s = openSocket(can_bus);

    // Create the writer and the listener 
    m_writer = new Writer(m_motors, ids, s);
    m_listener = new Listener(m_motors, ids, s);
}

MotorHandler::~MotorHandler()
{
    delete m_listener;
    delete m_writer;

    // close socket
}

int MotorHandler::openSocket(const char* can_bus)
{
    int s = socket(PF_CAN, SOCK_RAW|SOCK_NONBLOCK, CAN_RAW);
    if (s < 0) {
        cout << "Error opening the socket! Exiting" << endl;
        exit(1);
    }
    else   
        cout << "Socket opened successfully" << endl;

    struct sockaddr_can addr;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_bus);
    ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));  // Init
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int result = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (result < 0) {
        cout << "Binding error! Exiting" << endl;
        exit(1);
    }
    else
        cout << "Binding ok " << endl;

    return s;
}

/*
 *****************************************************************************
 *                               Motor infos
 ****************************************************************************/

int MotorHandler::getModel(int id)
{
    m_writer->requestModel(id);

    return 1;
}
