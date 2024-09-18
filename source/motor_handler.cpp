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
    m_nbrMotors = m_ids.size();

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

// TIMEOUT???
void MotorHandler::pingMotors()
{
    for (int i=0; i<m_nbrMotors; i++)
        m_writer->requestModel(m_ids[i]);

    for (int i=0; i<m_nbrMotors; i++) {
        int id = m_ids[i];
        bool hasResponded = 0;
        char model[FRAME_LENGTH-1];

        // Wait for the model update
        while(hasResponded != 1)
            m_listener->getModel(id, hasResponded, model);

        if (hasResponded == 1) {
            cout << "Motor " << id << " pinged successfully. Model: ";
            for (int j=0; j<FRAME_LENGTH-1; j++)
                cout << model[j];
            cout << endl;
        }
        else {
            cout << "Error! Motor " << id << " is not responding" << endl;
            exit(1);
        }
    }
}


/*
 *****************************************************************************
 *                               Motor infos
 ****************************************************************************/

void MotorHandler::writeTorque(vector<float> torques) 
{
    writeTorque(m_ids, torques);
}


void MotorHandler::writeTorque(vector<int> ids, vector<float> torques) 
{
    for (int i=0; i<ids.size(); i++)
        m_writer->writeTorque(ids[i], torques[i]);
}

void MotorHandler::getTorqueFbck(vector<float>& torqueFbck)
{
    getTorqueFbck(m_ids, torqueFbck);
}

void MotorHandler::getTorqueFbck(vector<int> ids, vector<float>& torqueFbck)
{
    for (int i=0; i<ids.size(); i++)
        m_writer->requestMotorFbck(ids[i]);

    for (int i=0; i<ids.size(); i++)
        torqueFbck[i] = m_listener->getTorque(ids[i]);
}

void MotorHandler::writeSpeed(vector<float> speeds) 
{
    writeSpeed(m_ids, speeds);
}


void MotorHandler::writeSpeed(vector<int> ids, vector<float> speeds) 
{
    for (int i=0; i<ids.size(); i++)
        m_writer->writeSpeed(ids[i], speeds[i]);
}

void MotorHandler::getSpeedFbck(vector<float>& speedFbck)
{
    getSpeedFbck(m_ids, speedFbck);
}

void MotorHandler::getSpeedFbck(vector<int> ids, vector<float>& speedFbck)
{
    for (int i=0; i<ids.size(); i++)
        m_writer->requestMotorFbck(ids[i]);

    for (int i=0; i<ids.size(); i++)
        speedFbck[i] = m_listener->getSpeed(ids[i]);
}
