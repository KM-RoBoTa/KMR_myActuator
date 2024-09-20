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

    usleep(2*1000000);
    pingMotors();
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


void MotorHandler::pingMotors()
{
    vector<string> models(m_nbrMotors);
    bool success = getModel(m_ids, models);
    
    cout << endl;
    for (int i=0; i<m_nbrMotors; i++)
    {
        if (!models[i].empty())
            cout << "Motor " << m_ids[i] << " pinged successfully! Model: " << models[i] << endl;
        else {
            cout << "Error! Motor " << m_ids[i] << " is not responding" << endl;
            exit(1);
        }  
    }
    cout << endl;
}


/*
 *****************************************************************************
 *                               Motor infos
 ****************************************************************************/

// --------- PID ----------- //
bool MotorHandler::getPID(vector<int> ids, vector<PIDReport>& pidReports)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestPID(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send PID-reading for motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        PIDReport tempPacket;
        bool success = m_listener->getPID(ids[i], tempPacket);
        if (success)
            pidReports[i] = tempPacket;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;
}

bool MotorHandler::getPID(vector<PIDReport>& pidReports)
{
    return(getPID(m_ids, pidReports));
}


bool MotorHandler::writePID_RAM(vector<int> ids, vector<PIDReport> pidReports)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->writePID_RAM(ids[i], pidReports[i]) < 0)
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

bool MotorHandler::writePID_RAM(vector<PIDReport> pidReports)
{
    return(writePID_RAM(m_ids, pidReports));
}

bool MotorHandler::writePID_EEPROM(vector<int> ids, vector<PIDReport> pidReports)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->writePID_EEPROM(ids[i], pidReports[i]) < 0)
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

bool MotorHandler::writePID_EEPROM(vector<PIDReport> pidReports)
{
    return(writePID_EEPROM(m_ids, pidReports));
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


// --------- Status and error reports  ----------- //

bool MotorHandler::getErrorReport(vector<int> ids, vector<ErrorReport>& errorReports)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestErrorReport(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send error report request for motor " << ids[i] << endl;
    }    

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        ErrorReport temp;
        bool success = m_listener->getErrorReport(ids[i], temp);
        if (success)
            errorReports[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;                
} 

bool MotorHandler::getErrorReport(vector<ErrorReport>& errorReports)
{
    return(getErrorReport(m_ids, errorReports));
}

bool MotorHandler::getPhaseReport(vector<int> ids, vector<PhaseReport>& phaseReports)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestPhaseReport(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send phase report request for motor " << ids[i] << endl;
    }    

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        PhaseReport temp;
        bool success = m_listener->getPhaseReport(ids[i], temp);
        if (success)
            phaseReports[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;                
} 

bool MotorHandler::getPhaseReport(vector<PhaseReport>& phaseReports)
{
    return(getPhaseReport(m_ids, phaseReports));
}


bool MotorHandler::getTorqueFbck(vector<int> ids, vector<float>& torqueFbck)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestMotorFbck(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send feedback request to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        float temp = 0;
        bool success = m_listener->getTorque(ids[i], temp);
        if (success)
            torqueFbck[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;      
}

bool MotorHandler::getTorqueFbck(vector<float>& torqueFbck)
{
    return(getTorqueFbck(m_ids, torqueFbck));
}


bool MotorHandler::getSpeedFbck(vector<int> ids, vector<float>& speedFbck)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestMotorFbck(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send feedback request to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        float temp = 0;
        bool success = m_listener->getSpeed(ids[i], temp);
        if (success)
            speedFbck[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;      
}

bool MotorHandler::getSpeedFbck(vector<float>& speedFbck)
{
    return(getSpeedFbck(m_ids, speedFbck));
}


bool MotorHandler::getTemperatureFbck(vector<int> ids, vector<int>& tempFbck)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestMotorFbck(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send feedback request to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        int temp = 0;
        bool success = m_listener->getTemperature(ids[i], temp);
        if (success)
            tempFbck[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;      
}

bool MotorHandler::getTemperatureFbck(vector<int>& tempFbck)
{
    return(getTemperatureFbck(m_ids, tempFbck));
}

bool MotorHandler::getFullFbck(vector<int> ids, vector<StatusReport>& statusReport)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestMotorFbck(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send feedback request to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        StatusReport temp;
        bool success = m_listener->getFullStatusReport(ids[i], temp);
        if (success)
            statusReport[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;     
}

bool MotorHandler::getFullFbck(std::vector<StatusReport>& statusReport)
{
    return(getFullFbck(m_ids, statusReport));
}


// ----------  Commands ----------- //

bool MotorHandler::writeTorque(vector<int> ids, vector<float> torques) 
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->writeTorque(ids[i], torques[0]) < 0)
            cout << "[FAILED REQUEST] Failed to send torque command to motor " << ids[i] << endl;
    }    

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->torque_command_received(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::writeTorque(vector<float> torques) 
{
    return(writeTorque(m_ids, torques));
}


bool MotorHandler::writeSpeed(vector<int> ids, vector<float> speeds) 
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->writeSpeed(ids[i], speeds[0]) < 0)
            cout << "[FAILED REQUEST] Failed to send speed command to motor " << ids[i] << endl;
    } 

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->speedWritten(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}


bool MotorHandler::writeSpeed(vector<float> speeds) 
{
    return(writeSpeed(m_ids, speeds));
}

bool MotorHandler::writeMotion(vector<int> ids, vector<float> pos, vector<float> speeds,
                 vector<float> Kps, vector<float> Kds, vector<float> Tff)
{
     for (int i=0; i<ids.size(); i++) {
        if(m_writer->writeMotionMode(ids[i], pos[i], speeds[i], Kps[i], Kds[i], Tff[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send motion command to motor " << ids[i] << endl;
    } 

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->motion_written(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::writeMotion(vector<float> pos, vector<float> speeds,
                 vector<float> Kps, vector<float> Kds, vector<float> Tffs)
{
    return(writeMotion(m_ids, pos, speeds, Kps, Kds, Tffs));
}


// ----------  Motor info ----------- //

bool MotorHandler::getModel(vector<int> ids, vector<std::string>& models)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestModel(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send model request to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->getModel(ids[i], models[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;      
}
