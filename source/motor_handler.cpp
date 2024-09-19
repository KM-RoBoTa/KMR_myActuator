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

// --------- PID ----------- //
bool MotorHandler::getPID(vector<int> ids, vector<PacketPID>& vecPID)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestPID(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send PID-reading for motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        PacketPID tempPacket;
        bool success = m_listener->getPID(ids[i], tempPacket);
        if (success)
            vecPID[i] = tempPacket;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;
}

bool MotorHandler::getPID(vector<PacketPID>& vecPID)
{
    return(getPID(m_ids, vecPID));
}


bool MotorHandler::writePID_RAM(vector<int> ids, vector<PacketPID> vecPID)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->writePID_RAM(ids[i], vecPID[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send PID-writing to RAM for motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->PID_written_RAM(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;    
}

bool MotorHandler::writePID_RAM(vector<PacketPID> vecPID)
{
    return(writePID_RAM(m_ids, vecPID));
}

bool MotorHandler::writePID_EEPROM(vector<int> ids, vector<PacketPID> vecPID)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->writePID_EEPROM(ids[i], vecPID[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send PID-writing to EEPROM for motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->PID_written_EEPROM(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;   

    //RESET MOTORS???
}

bool MotorHandler::writePID_EEPROM(vector<PacketPID> vecPID)
{
    return(writePID_EEPROM(m_ids, vecPID));
}


// --------- Acc settings  ----------- //

bool MotorHandler::getAccelerationSettings(vector<int> ids, ACC_SETTINGS setting, vector<int>& accs)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestAccSettings(ids[i], setting) < 0)
            cout << "[FAILED REQUEST] Failed to send acc. reading for motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        int temp;
        bool success = m_listener->getAccSettings(ids[i], setting, temp);
        if (success)
            accs[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;           
}

bool MotorHandler::getAccelerationSettings(ACC_SETTINGS setting, vector<int>& accs)
{
    return(getAccelerationSettings(m_ids, setting, accs));
}

bool MotorHandler::writeAccelerationSettings(vector<int> ids, ACC_SETTINGS setting, vector<int> accs)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->writeAccSettings(ids[i], setting, accs[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send acc-writing to EEPROM for motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->accSettingWritten(ids[i], setting);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;            
}

bool MotorHandler::writeAccelerationSettings(ACC_SETTINGS setting, vector<int> accs)
{
    return(writeAccelerationSettings(m_ids, setting, accs));
}


// --------- On/off  ----------- //

// Stops speed + torque. Doesn't write to EEPROM (to check)
bool MotorHandler::shutdownMotors(std::vector<int> ids)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestShutdown(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send shutdown for motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->shutdown_received(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;            
}

// Stops speed + torque. Doesn't write to EEPROM (to check)
bool MotorHandler::shutdownMotors()
{
    return(shutdownMotors(m_ids));
}

// Stops speed, keeps torque
bool MotorHandler::stopMotors(std::vector<int> ids)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestStop(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send stop for motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->stop_received(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;            
}

// Stops speed, keeps torque
bool MotorHandler::stopMotors()
{
    return(stopMotors(m_ids));
}

// Need some sleep time after the reset
bool MotorHandler::resetMotors(std::vector<int> ids)
{
    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestReset(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send reset for motor " << ids[i] << endl;
        else
            fullSuccess += 1;
    }

    // Since the motor is resetting, no feedback

    // If no problem sending to any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;   

}

// Stops speed, keeps torque
bool MotorHandler::resetMotors()
{
    return(resetMotors(m_ids));
}

bool MotorHandler::releaseBrake(vector<int> ids)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestBrakeRelease(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send brake release for motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->brake_release_received(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;            
}

bool MotorHandler::releaseBrake()
{
    return(releaseBrake(m_ids));          
}

bool MotorHandler::lockBrake(vector<int> ids)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestBrakeLock(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send brake lock for motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->brake_lock_received(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;            
}

bool MotorHandler::lockBrake()
{
    return(lockBrake(m_ids));          
}








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

    for (int i=0; i<m_ids.size(); i++)
        bool done = m_listener->speedWritten(m_ids[i]);
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
