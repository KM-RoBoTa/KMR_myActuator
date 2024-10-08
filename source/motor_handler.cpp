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

#define RESET_SPEED 2*M_PI/4

using namespace std;

MotorHandler::MotorHandler(vector<int> ids, const char* can_bus, vector<Model> models)
{
    if (models.size() != ids.size()) {
        cout << "Error! The size of the models' vector is not equal to the size of motor ids" << endl;
        exit(1);
    }

    m_ids = ids;
    m_nbrMotors = m_ids.size();

    for (int i=0; i<ids.size(); i++) {
        Motor* motor = new Motor(ids[i], models[i]);
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

    for (int i=0; i<m_nbrMotors; i++)
        delete m_motors[i];

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
    vector<float> runtimes(m_nbrMotors, 0);
    bool success = getRuntime(m_ids, runtimes);
    
    cout << endl;
    for (int i=0; i<m_nbrMotors; i++)
    {
        if (runtimes[i] != 0)
            cout << "Motor " << m_ids[i] << " pinged successfully!" << endl;
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
/*
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

bool MotorHandler::getModel(vector<std::string>& models)
{
    return(getModel(m_ids, models));
}*/


bool MotorHandler::getOperatingMode(vector<int> ids, vector<OperatingMode>& modes)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestOperatingMode(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send operating mode request to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    OperatingMode temp;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->getOperatingMode(ids[i], temp);
        if (success)
            modes[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::getOperatingMode(vector<OperatingMode>& modes)
{
    return(getOperatingMode(m_ids, modes));
}

bool MotorHandler::getPowerConsumption(vector<int> ids, vector<float>& powers)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestPowerConsumption(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send power consumption request to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    float temp;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->getPowerConsumption(ids[i], temp);
        if (success)
            powers[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::getPowerConsumption(vector<float>& powers)
{
    return(getPowerConsumption(m_ids, powers));
}


bool MotorHandler::getRuntime(vector<int> ids, vector<float>& runtimes)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestRuntime(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send runtime request to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    float temp;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->getRuntime(ids[i], temp);
        if (success)
            runtimes[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::getRuntime(vector<float>& runtimes)
{
    return(getRuntime(m_ids, runtimes));
}

bool MotorHandler::getSoftwareDate(vector<int> ids, vector<int>& dates)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestSoftwareDate(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send software date request to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    int temp;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->getSoftwareDate(ids[i], temp);
        if (success)
            dates[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::getSoftwareDate(vector<int>& dates)
{
    return(getSoftwareDate(m_ids, dates));
}

// ----------  Other settings ----------- //


bool MotorHandler::enableCANFilter(std::vector<int> ids){
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->enableCANFilter(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send CAN filter enable to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->canFilterWritten(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::enableCANFilter()
{
    return(enableCANFilter(m_ids));
}

bool MotorHandler::disableCANFilter(std::vector<int> ids)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->disableCANFilter(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send CAN filter disable to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->canFilterWritten(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}


bool MotorHandler::disableCANFilter()
{
    return(disableCANFilter(m_ids));
}


// ----------  Other settings ----------- //

bool MotorHandler::resetMultiturnCounter(vector<int> ids)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestClearMultiturn(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send multiturn reset request to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->multiturnResetWritten(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::resetMultiturnCounter()
{
    return(m_ids, resetMultiturnCounter(m_ids));
}

bool MotorHandler::enableActiveErrorFbck(std::vector<int> ids)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->enableActiveErrorFbck(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send active error status to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->activeErrorFbckWritten(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::enableActiveErrorFbck()
{
    return(enableActiveErrorFbck(m_ids));
}


bool MotorHandler::disableActiveErrorFbck(std::vector<int> ids)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->disableActiveErrorFbck(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send inactive error status to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->activeErrorFbckWritten(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::disableActiveErrorFbck()
{
    return(disableActiveErrorFbck(m_ids));
}

// NEED RESET?
bool MotorHandler::setMultiturnMode(std::vector<int> ids)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->setMultiturnMode(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send multiturn mode to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->multiturnModeWritten(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

// NEED RESET?
bool MotorHandler::setMultiturnMode()
{
    return(setMultiturnMode(m_ids));
}

// NEED RESET?
bool MotorHandler::setSingleturnMode(std::vector<int> ids)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->setSingleturnMode(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send singleturn mode to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->multiturnModeWritten(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

// NEED RESET?
bool MotorHandler::setSingleturnMode()
{
    return(setSingleturnMode(m_ids));
}

// ---------- Position feedbacks ----------- //


bool MotorHandler::getEncoderPosition(vector<int> ids, vector<int32_t>& positions)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestEncoderPosition(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to request encoder position to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        int32_t temp;
        bool success = m_listener->getEncoderPosition(ids[i], temp);
        if (success)
            positions[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::getEncoderPosition(vector<int32_t>& positions)
{
    return(getEncoderPosition(m_ids, positions));
}

bool MotorHandler::getRawEncoderPosition(vector<int> ids, vector<uint32_t>& positions)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestRawEncoderPosition(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to request raw encoder position to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        uint32_t temp;
        bool success = m_listener->getRawEncoderPosition(ids[i], temp);
        if (success)
            positions[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}


bool MotorHandler::getRawEncoderPosition(vector<uint32_t>& positions)
{
    return(getRawEncoderPosition(m_ids, positions));
}

bool MotorHandler::getEncoderZeroOffset(vector<int> ids, vector<uint32_t>& positions)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestEncoderZeroOffset(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to request encoder zero position to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        uint32_t temp;
        bool success = m_listener->getEncoderZeroOffset(ids[i], temp);
        if (success)
            positions[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::getEncoderZeroOffset(vector<uint32_t>& positions)
{
    return(getEncoderZeroOffset(m_ids, positions));
}

bool MotorHandler::writeEncoderZeroOffset(vector<int> ids, vector<uint32_t> offsets)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->writeEncoderZeroOffset(ids[i], offsets[i]) < 0)
            cout << "[FAILED REQUEST] Failed to write encoder zero position to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->encoderZeroOffsetWritten(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::writeEncoderZeroOffset(vector<uint32_t> offsets)
{
    return(writeEncoderZeroOffset(m_ids, offsets));
}

bool MotorHandler::writeEncoderZeroOffset_currentPos(vector<int> ids)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->writeEncoderZeroOffset(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to write encoder zero position to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->encoderZeroOffsetWritten(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::writeEncoderZeroOffset_currentPos()
{
    return(writeEncoderZeroOffset_currentPos(m_ids));
}

bool MotorHandler::getEncoderFbck_ST(vector<int> ids, vector<Encoder_ST>& encoders)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestEncoderFbck_ST(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to request ST encoder fbck to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        Encoder_ST temp;
        bool success = m_listener->getEncoderFbck_ST(ids[i], temp);
        if (success)    
            encoders[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::getEncoderFbck_ST(vector<Encoder_ST>& encoders)
{
    return(getEncoderFbck_ST(m_ids, encoders));
}

bool MotorHandler::getPosition_MT(vector<int> ids, vector<float>& angles)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestPosition_MT(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to request MT position fbck to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        float temp;
        bool success = m_listener->getPosition_MT(ids[i], temp);

        // Custom offset correction (no effect by default)
        int idx = getIndex(m_ids, ids[i]);
        temp = temp - m_motors[idx]->refOffset;

        if (success)    
            angles[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::getPosition_MT(vector<float>& angles)
{
    return(getPosition_MT(m_ids, angles));
}

bool MotorHandler::getPosition_ST(vector<int> ids, vector<float>& angles)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->requestPosition_ST(ids[i]) < 0)
            cout << "[FAILED REQUEST] Failed to request ST position fbck to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        float temp;
        bool success = m_listener->getPosition_ST(ids[i], temp);
        if (success)    
            angles[i] = temp;
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::getPosition_ST(vector<float>& angles)
{
    return(getPosition_ST(m_ids, angles));
}

bool MotorHandler::writePosition_MT(vector<int> ids, vector<float> maxSpeeds, vector<float> angles)
{
    for (int i=0; i<ids.size(); i++) {
        int idx = getIndex(m_ids, ids[i]);

        // Saturate the input angle if angle limitation is active
        if (m_motors[idx]->limitAngles) {
            float min = m_motors[idx]->minAngle;
            float max = m_motors[idx]->maxAngle;
            angles[i] = saturate(min, max, angles[i]);
        }

        // Custom offset correction (no effect by default)
        angles[i] += m_motors[idx]->refOffset;

        if(m_writer->writePosition_MT(ids[i], maxSpeeds[i], angles[i]) < 0)
            cout << "[FAILED REQUEST] Failed to write MT position to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->positionMT_written(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;    
}


bool MotorHandler::writePosition_MT(vector<float> maxSpeeds, vector<float> angles)
{
    return(writePosition_MT(m_ids, maxSpeeds, angles));
}


bool MotorHandler::writePosition_ST(vector<int> ids, vector<float> maxSpeeds, vector<float> angles)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->writePosition_ST(ids[i], maxSpeeds[i], angles[i]) < 0)
            cout << "[FAILED REQUEST] Failed to write ST position to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->positionST_written(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;    
}


bool MotorHandler::writePosition_ST(vector<float> maxSpeeds, vector<float> angles)
{
    return(writePosition_ST(m_ids, maxSpeeds, angles));
}


bool MotorHandler::writePositionIncrement_MT(std::vector<int> ids, std::vector<float> maxSpeeds, std::vector<float> angles)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->writePositionIncrement_MT(ids[i], maxSpeeds[i], angles[i]) < 0)
            cout << "[FAILED REQUEST] Failed to write MT position increment to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        bool success = m_listener->positionIncrMT_written(ids[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::writePositionIncrement_MT(vector<float> maxSpeeds, vector<float> angles)
{
    return(writePositionIncrement_MT(m_ids, maxSpeeds, angles));
}

// --------------------------------------------
// ---------- Custom single turn ----------- //
// --------------------------------------------

// Set custom working point
void MotorHandler::setCustomWorkingPoint(std::vector<int> ids)
{
    for (int i=0; i<ids.size(); i++) {
        // Read position for each ID
        vector<float> positions(1,0);
        bool success = getPosition_MT(positions);
        if (!success) {
            cout << "Error! Could not calculate the working point for motor "<< ids[i] << ". Exiting" << endl;
            exit(1);
        }
        float angle = positions[0];

        // ---- Get multiturn offset -----//

        // Modulo 2PI, which brings the angle between -2pi and 2pi
        int k = (int)( (float)angle /(float)(2*M_PI) );
        angle = angle-k*2*M_PI;

        // Additional increment of k to bring the angle between -pi and pi
        if (angle > M_PI)
            k++;
        else if (angle < -M_PI)
            k--;

        // Save the parameters to the motor
        int idx = getIndex(m_ids, ids[i]);
        m_motors[idx]->k += k;            
        m_motors[idx]->refOffset += 2*k*M_PI;
    }
}

// Set custom working point
void MotorHandler::setCustomWorkingPoint()
{
    setCustomWorkingPoint(m_ids);
}

void MotorHandler::setCustomSingleTurn(std::vector<int> ids)
{
    for (int i=0; i<ids.size(); i++) {
	    int idx = getIndex(m_ids, ids[i]);

        // Set min/max angles to +-pi only if no valid limits have been previously set with setPositionLimits()
        if(m_motors[idx]->minAngle < -M_PI)
            m_motors[idx]->minAngle = -M_PI;

        if (m_motors[idx]->maxAngle > M_PI)
            m_motors[idx]->maxAngle = M_PI;

        // Set settings flags
        m_motors[idx]->customST = 1;
        m_motors[idx]->limitAngles = 1;       
    }
}

void MotorHandler::setCustomSingleTurn()
{
    setCustomSingleTurn(m_ids);
}

// For custom single turn
void MotorHandler::setPositionLimits(vector<int> ids, vector<float> minAngles, vector<float> maxAngles)
{
    for (int i=0; i<ids.size(); i++) {
        if (minAngles[i] > maxAngles[0]) {
            cout << "Error! Trying to set minimal angle bigger than the maximal one for motor "
            << ids[i] << ". Exiting" << endl;
            exit(1);
        }

	    int idx = getIndex(m_ids, ids[i]);

        // If motor previously set as single turn, saturate angles as a security
        if (m_motors[idx]->customST) {
            if (maxAngles[i] > M_PI) {
                cout << "Motor " << ids[i] << " has been previously set as single turn. "
                "Setting max angle to +PI instead of the requested " << maxAngles[i] << endl; 
                maxAngles[i] = M_PI;

            }
            if (minAngles[i] < -M_PI) {
                cout << "Motor " << ids[i] << " has been previously set as single turn. "
                "Setting min angle to -PI instead of the requested " << minAngles[i] << endl; 
                minAngles[i] = -M_PI;
            }
        }

        m_motors[idx]->minAngle = minAngles[i];
        m_motors[idx]->maxAngle = maxAngles[i];
        m_motors[idx]->limitAngles = 1;
    }
}

// For custom single turn
void MotorHandler::setPositionLimits(vector<float> minAngles, vector<float> maxAngles)
{
    setPositionLimits(m_ids, minAngles, maxAngles);
}


void MotorHandler::resetCustomMultiturn(std::vector<int> ids)
{
    // Update the working point
    setCustomWorkingPoint(ids);

    // Go to the 0-position
    vector<float> zeroes(ids.size(), 0);
    vector<float> speeds(ids.size(), RESET_SPEED);
    writePosition_MT(ids, speeds, zeroes);
    usleep(2*1000000);

    // Reset the motor's multiturn and reset them
    setMultiturnMode(ids);
    resetMotors(ids);
    usleep(1*1000000);
}


void MotorHandler::resetCustomMultiturn()
{
    resetCustomMultiturn(m_ids);
}