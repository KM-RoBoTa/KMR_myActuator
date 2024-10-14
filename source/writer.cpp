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
#include <cmath>

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

	// Create the protocol-specific writers
    m_p1Writer = new P1Writer(motors, ids, s);
    m_p2Writer = new P2Writer(motors, ids, s);
}

/**
 * @brief	Class destructor
 */
Writer::~Writer()
{
    delete m_p1Writer;
    delete m_p2Writer;
}

/*
 *****************************************************************************
 *                               Motor infos
 ****************************************************************************/

// --------- PID ----------- //

int Writer::requestPID(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestPID(id);
    else {
        cout << "request PID protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}


int Writer::writePID_RAM(int id, PIDReport pidReport)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->writePID_RAM(id, pidReport);
    else {
        cout << "write PID RAM protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}


int Writer::writePID_EEPROM(int id, PIDReport pidReport)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->writePID_EEPROM(id, pidReport);
    else {
        cout << "write PID EEPROM protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

// --------- Acc settings  ----------- //

int Writer::requestAccSettings(int id, ACC_SETTINGS setting)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestAccSettings(id, setting);
    else {
        cout << "Request Acc settings protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

// Writes to EEPROM. Value between 1.74 and 1047 rad/s²
int Writer::writeAccSettings(int id, ACC_SETTINGS setting, int value)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->writeAccSettings(id, setting, value);
    else {
        cout << "Write acc settings protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

// --------- On/off ----------- //

int Writer::requestShutdown(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestShutdown(id);
    else {
        cout << "Request shutdown protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::requestStop(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestStop(id);
    else {
        cout << "Request stop protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }
    return nbytes;
}

int Writer::requestReset(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestReset(id);
    else {
        cout << "Request reset protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::requestBrakeRelease(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestBrakeRelease(id);
    else {
        cout << "Request break release protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::requestBrakeLock(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestBrakeLock(id);
    else {
        cout << "Request brake lock protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}


// --------- Status and errors ----------- //

int Writer::requestErrorReport(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestErrorReport(id);
    else {
        cout << "Request error report protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::requestMotorFbck(int id)
{   
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestMotorFbck(id);
    else {
        cout << "Request motor fbck protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}   


int Writer::requestPhaseReport(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestPhaseReport(id);
    else {
        cout << "Request phase report protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

// --------- Commands ----------- //

int Writer::writeTorque(int id, float torque)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->writeTorque(id, torque);
    else {
        cout << "Request write torque protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}


int Writer::writeSpeed(int id, float speed)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->writeSpeed(id, speed);
    else {
        cout << "Request write speed protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}


int Writer::writeHybrid(int id, float pos, float speed, float Kp, float Kd, float Tff)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->writeHybrid(id, pos, speed, Kp, Kd, Tff);
    else {
        cout << "Write motion mode protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}


// --------- Motor infos ----------- //

int Writer::requestModel(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestModel(id);
    else {
        cout << "Request model protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;  
}

int Writer::requestOperatingMode(int id) 
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestOperatingMode(id);
    else {
        cout << "Request operating mode protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes; 
}

int Writer::requestPowerConsumption(int id) 
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestPowerConsumption(id);
    else {
        cout << "Request power consumption protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::requestRuntime(int id) 
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestRuntime(id);
    else {
        cout << "Request runtime protocol 2" << endl;    
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::requestSoftwareDate(int id) 
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestSoftwareDate(id);
    else {
        cout << "Request software date protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::enableCANFilter(int id) 
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->enableCANFilter(id);
    else {
        cout << "Enable CAN filter protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::disableCANFilter(int id) 
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->disableCANFilter(id);
    else {
        cout << "Disable CAN filter protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}


// NEED TO RESTART. Resets multiturn, updates 0 and saves to EEPROM
int Writer::requestClearMultiturn(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestClearMultiturn(id);
    else {
        nbytes = MSG_UNSUPPORTED_P2;
        cout << "Request clear multiturn protocol 2" << endl;
    }

    return nbytes;
}

int Writer::enableActiveErrorFbck(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->enableActiveErrorFbck(id);
    else {
        cout << "Enable active error fbck protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::disableActiveErrorFbck(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->disableActiveErrorFbck(id);
    else {
        cout << "Disable active error fbck protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

/// DONE
int Writer::setMultiturnMode(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->setMultiturnMode(id);
    else
        nbytes = m_p2Writer->setMultiturnMode(id);

    return nbytes;
}

int Writer::setSingleturnMode(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->setSingleturnMode(id);
    else {
        cout << "Set single turn mode protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::requestEncoderPosition(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestEncoderPosition(id);
    else {
        cout << "Request encoder position protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::requestRawEncoderPosition(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestRawEncoderPosition(id);
    else {
        cout << "Request raw encoder position protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}


int Writer::requestEncoderZeroOffset(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestEncoderZeroOffset(id);
    else {
        cout << "Request encoder zero offset protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::writeEncoderZeroOffset(int id, uint32_t offset)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->writeEncoderZeroOffset(id, offset);
    else {
        cout << "Write encoder zero offset protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::writeEncoderZeroOffset(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->writeEncoderZeroOffset(id);
    else {
        cout << "Write encoder zero offset protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::requestEncoderFbck_ST(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestEncoderFbck_ST(id);
    else {
        cout << "Request encoder fbck ST protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes; 
}

int Writer::requestPosition_MT(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestPosition_MT(id);
    else {
        cout << "Request position MT protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::requestPosition_ST(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->requestPosition_ST(id);
    else {
        cout << "Request position ST protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}    

int Writer::writePosition_MT(int id, float maxSpeed, float angle)
{   
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->writePosition_MT(id, maxSpeed, angle);
    else {
        cout << "Write position MT protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}


int Writer::writePosition_ST(int id, float maxSpeed, float angle)
{   
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->writePosition_ST(id, maxSpeed, angle);
    else {
        cout << "Write position ST protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}

int Writer::writePositionIncrement_MT(int id, float maxSpeed, float angle)
{   
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1)
        nbytes = m_p1Writer->writePositionIncrement_MT(id, maxSpeed, angle);
    else {
        cout << "Write position increment MT protocol 2" << endl;
        nbytes = MSG_UNSUPPORTED_P2;
    }

    return nbytes;
}


/*
 *****************************************************************************
 *                              PROTOCOL 2
 ****************************************************************************/

/// DONE
int Writer::setDefaultCommandType(int id)
{
    int idx = getIndex(m_ids, id);
    int nbytes = MSG_FAIL;    
    if (m_motors[idx]->protocol == PROTOCOL_1) {
        cout << "Setting default command type unsupported by protocol 1 (motor " << id << ")" << endl;
        nbytes = MSG_UNSUPPORTED_P1;
    }
    else
        nbytes = m_p2Writer->setDefaultCommandType(id);

    return nbytes;
}