/**
 ****************************************************************************
 * @file        listener.cpp
 * @brief       CAN bus listener, running in own thread
 ****************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 09/2024
 * @authors  kamilo.melo@km-robota.com, 09/2024
 ****************************************************************************
 */

// Standard libraries
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <cmath>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <cstring>
#include <net/if.h> // network devices
#include <sstream> // Convert dec-hex

#include "listener.hpp"


using namespace std;


/**
 * @brief       Start the CAN bus listener
 * @param[in]   s Socket
 */
Listener::Listener(vector<Motor*> motors, vector<int> ids, int s)
{
	m_motors = motors;
	m_nbrMotors = motors.size();
    m_stopThread = false;
    m_thread = thread(&Listener::listenerLoop, this, s);
	m_ids = ids;

    cout << "Creating the CAN listener's thread..." << endl;
	usleep(50000);  
}

/**
 * @brief	Class destructor. Takes care of safely stopping the thread
 */
Listener::~Listener()
{
	// Change the internal boolean to get the thread out of its loop function
	{
		scoped_lock lock(m_mutex);
		m_stopThread = true;	
	}	
    
	// Block the main thread until the Listener thread finishes
    if (m_thread.joinable()) 
        m_thread.join();

	cout << "Listener thread safely stopped" << endl;
}

/**
 * @brief       
 * @param[in]   s Socket
 * @retval      1 when the thread is finished
 */
int Listener::listenerLoop(int s)
{
	bool stopThread = 0;

	// -----  Start of the main loop -----
	cout << "Starting CAN bus monitoring" << endl;
    while(!stopThread) {

        struct can_frame frame;
        int nbytes = read(s, &frame, sizeof(struct can_frame));

        if (nbytes > 0) {
			
            // Parse the received frame

			// TEMPORARY
			int function = frame.data[0];

			switch (function)
			{
			case 0x30:	parsePIDFbck(frame); break;
			case 0x31: 	parsePID_RAM_write(frame); break;
			case 0x32:	parsePID_EEPROM_write(frame); break;

			case 0x42:	parseAccSettingsFbck(frame); break;
			case 0x43:	parseAccSettings_write(frame); break;

			case 0xB5: 	readModel(frame);	break;
			case 0xA1:	parseTorqueCommand(frame); break;
			case 0xA2: 	parseSpeedCommand(frame); break;
			case 0x9C:	parseMotorFbck(frame); break;
			default: cout << "Unknown function" ; break;
			}
        }

		// Thread sleep for scheduling
		std::this_thread::sleep_for(chrono::microseconds(50));

		{
			scoped_lock lock(m_mutex);
			stopThread = m_stopThread;	
		}	
    }

    return 0;
}

/*
 *****************************************************************************
 *                             Interface with main thread
 ****************************************************************************/

// --------- PID ----------- //

bool Listener::getPID(int id, PacketPID& packetPID)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] PID reading of motor " << id << " timed out!" << endl;
			break;
		}

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_PID;
		packetPID.Kp_torque = m_motors[idx]->Kp_torque;
		packetPID.Ki_torque = m_motors[idx]->Ki_torque;
		packetPID.Kp_speed = m_motors[idx]->Kp_speed;
		packetPID.Ki_speed = m_motors[idx]->Ki_speed;
		packetPID.Kp_pos = m_motors[idx]->Kp_pos;
		packetPID.Ki_pos = m_motors[idx]->Ki_pos;

		// Clear update flag
		m_motors[idx]->fr_PID = 0;
	}

	return available;	
}

bool Listener::PID_written_RAM(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] PID RAM writing of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fw_PID_RAM;

		// Clear update flag
		m_motors[idx]->fw_PID_RAM = 0;
	}

	return available;	
}

bool Listener::PID_written_EEPROM(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] PID EEPROM writing of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fw_PID_EEPROM;

		// Clear update flag
		m_motors[idx]->fw_PID_EEPROM = 0;
	}

	return available;	
}

// --------- Acc settings  ----------- //

bool Listener::getAccSettings(int id, ACC_SETTINGS setting, int& acc)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting the acc. setting of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		switch (setting)
		{
		case POSITION_ACC:
			acc = m_motors[idx]->positionAcc;
			available = m_motors[idx]->fr_posAcc;
			m_motors[idx]->fr_posAcc = 0;
			break;
		case POSITION_DEC:
			acc = m_motors[idx]->positionDec;
			available = m_motors[idx]->fr_posDec;
			m_motors[idx]->fr_posDec = 0;
			break;
		case SPEED_ACC:
			acc = m_motors[idx]->speedAcc;
			available = m_motors[idx]->fr_speedAcc;
			m_motors[idx]->fr_speedAcc = 0;
			break;
		case SPEED_DEC:
			acc = m_motors[idx]->speedDec;
			available = m_motors[idx]->fr_speedDec;
			m_motors[idx]->fr_speedDec = 0;
			break;	
		default:
			cout << "Error! Listener did not recognize the received acc parameter" << endl;
			exit(1); 
			break;
		}
	}

	return available;	
}

bool Listener::accSettingWritten(int id, ACC_SETTINGS setting)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Acc. EEPROM writing of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		switch (setting)
		{
		case POSITION_ACC:
			available = m_motors[idx]->fw_posAcc;
			m_motors[idx]->fw_posAcc = 0;
			break;
		case POSITION_DEC:
			available = m_motors[idx]->fw_posDec;
			m_motors[idx]->fw_posDec = 0;
			break;
		case SPEED_ACC:
			available = m_motors[idx]->fw_speedAcc;
			m_motors[idx]->fw_speedAcc = 0;
			break;
		case SPEED_DEC:
			available = m_motors[idx]->fw_speedDec;
			m_motors[idx]->fw_speedDec = 0;
			break;	
		default:
			cout << "Error! Listener did not recognize the received acc parameter" << endl;
			exit(1); 
			break;
		}
	}

	return available;		
}










bool Listener::speedWritten(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	while (available != 1) {
		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fw_speed;

		// Clear update flag
		m_motors[idx]->fw_speed = 0;
	}

	return available;		
}


void Listener::getModel(int id, bool& hasResponded, char model[])
{
	int idx = getIndex(m_ids, id);

	scoped_lock lock(m_mutex);
	hasResponded = m_motors[idx]->f_model;

	for (int i=0; i<FRAME_LENGTH-1; i++)
		model[i] = m_motors[idx]->model[i];

	// Clear update flag
	m_motors[idx]->f_model = 0;
}

float Listener::getTorque(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;
	float torque = 0;

	while (available != 1) {
		scoped_lock lock(m_mutex);
		available = m_motors[idx]->f_torque;
		torque = m_motors[idx]->torque;

		// Clear update flag
		m_motors[idx]->f_torque = 0;
	}

	return torque;
}

float Listener::getSpeed(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;
	float speed = 0;

	while (available != 1) {
		scoped_lock lock(m_mutex);
		available = m_motors[idx]->f_speed;
		speed = m_motors[idx]->speed;

		// Clear update flag
		m_motors[idx]->f_speed = 0;
	}

	return speed;
}

float Listener::getAngle(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;
	float angle = 0;

	while (available != 1) {
		scoped_lock lock(m_mutex);
		available = m_motors[idx]->f_angle;
		angle = m_motors[idx]->angle;

		// Clear update flag
		m_motors[idx]->f_angle = 0;
	}

	return angle;
}

float Listener::getTemperature(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;
	float temperature = 0;

	while (available != 1) {
		scoped_lock lock(m_mutex);
		available = m_motors[idx]->f_temperature;
		temperature = m_motors[idx]->temperature;

		// Clear update flag
		m_motors[idx]->f_temperature = 0;
	}

	return temperature;
}
/*
 *****************************************************************************
 *                               Motor infos
 ****************************************************************************/

// --------- PID ----------- //

// Reads currently used PID, so it could have been changed to RAM
void Listener::parsePIDFbck(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	// Write the read data into the corresponding place
	scoped_lock lock(m_mutex);
	m_motors[idx]->Kp_torque = frame.data[2];
	m_motors[idx]->Ki_torque = frame.data[3];
	m_motors[idx]->Kp_speed = frame.data[4];
	m_motors[idx]->Ki_speed = frame.data[5];
	m_motors[idx]->Kp_pos = frame.data[6];
	m_motors[idx]->Ki_pos = frame.data[7];

	// Set update flag
	m_motors[idx]->fr_PID = 1;
}

void Listener::parsePID_RAM_write(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Set up confirmation flag
	scoped_lock lock(m_mutex);
	m_motors[idx]->fw_PID_RAM= 1;	
}

void Listener::parsePID_EEPROM_write(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Set up confirmation flag
	scoped_lock lock(m_mutex);
	m_motors[idx]->fw_PID_EEPROM = 1;	
}

// --------- Acc settings  ----------- //


// Between 100 and 60000 deg/s² (= 1.74 to 1047 rad/s²)
void Listener::parseAccSettingsFbck(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	int mode = frame.data[1];
	int32_t accParameter = ( (int32_t) frame.data[7] << 24) +
						   ( (int32_t) frame.data[6] << 16) +
						   ( (int32_t) frame.data[5] <<  8) +
						   ( (int32_t) frame.data[4]      );
	int accValue = (int) accParameter; 

	// Convert to rad/s²
	float temp = deg2rad((float)accValue);
	accValue = (int) temp;

	// Write the read data into the corresponding place & update flags
	scoped_lock lock(m_mutex);
	switch (mode)
	{
	case 0x00:
		m_motors[idx]->positionAcc = accValue;
		m_motors[idx]->fr_posAcc = 1;
		break;
	case 0x01:
		m_motors[idx]->positionDec = accValue;
		m_motors[idx]->fr_posDec = 1;
		break;
	case 0x02:
		m_motors[idx]->speedAcc = accValue;
		m_motors[idx]->fr_speedAcc = 1;
		break;
	case 0x03:
		m_motors[idx]->speedDec = accValue;
		m_motors[idx]->fr_speedDec = 1;
		break;	
	default:
		cout << "Error! Listener did not recognize the received acc parameter" << endl;
		exit(1); 
		break;
	}
}

void Listener::parseAccSettings_write(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	int mode = frame.data[1];

	// Set up confirmation flag
	scoped_lock lock(m_mutex);
	switch (mode)
	{
	case 0x00:
		m_motors[idx]->fw_posAcc = 1;
		break;
	case 0x01:
		m_motors[idx]->fw_posDec = 1;
		break;
	case 0x02:
		m_motors[idx]->fw_speedAcc = 1;
		break;
	case 0x03:
		m_motors[idx]->fw_speedDec = 1;
		break;	
	default:
		cout << "Error! Listener did not recognize the received acc parameter" << endl;
		exit(1); 
		break;
	}
}










void Listener::readModel(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	// Write the read data into the corresponding place
	scoped_lock lock(m_mutex);
	for (int i=1; i<FRAME_LENGTH; i++)
		m_motors[idx]->model[i-1] = frame.data[i];

	// Set update flag
	m_motors[idx]->f_model = 1;
}

// 0xA1
void Listener::parseTorqueCommand(can_frame frame)
{
	// Extract the motor ID from the received frame
	/*int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	if (idx > 0) {

		// Calculate SI values from the packet
		int8_t temperatureParameter = frame.data[1];
		int16_t torqueParameter = ( (int16_t) frame.data[3] << 8) +
								  ( (int16_t) frame.data[2] );
		int16_t speedParameter = ( (int16_t) frame.data[5] << 8) +
							     ( (int16_t) frame.data[4] );
		int16_t angleParameter = ( (int16_t) frame.data[7] << 8) +
							     ( (int16_t) frame.data[6] );

		const float torqueUnit = 0.01;
		const float speedUnit = 1;
		const float posUnit = 1;
		int temperature = (int) temperatureParameter;
		float torque = torqueParameter * torqueUnit;
		float speed = speedParameter * speedUnit;
		float angle = angleParameter * posUnit;

		// Save the received values. TODO FROM HERE BELOW
		scoped_lock lock(m_mutex);
		for (int i=1; i<FRAME_LENGTH; i++)
			m_motors[idx]->model[i-1] = frame.data[i];

		// Set update flag
		m_motors[idx]->f_model = 1;	
	}
	else
		cout << "Error! Torque could not be read, unknown motor" << endl;*/
}

void Listener::parseSpeedCommand(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	// Set up confirmation flag
	scoped_lock lock(m_mutex);
	m_motors[idx]->fw_speed = 1;			
}

void Listener::parseMotorFbck(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	// Convert from parameters to SI values
	int8_t temperatureParameter = frame.data[1];
	int16_t torqueParameter = ( (int16_t) frame.data[3] << 8) 
								+ ( (int16_t) frame.data[2] );
	int16_t speedParameter = ( (int16_t) frame.data[5] << 8) 
							+ ( (int16_t) frame.data[4] );
	int16_t angleParameter = ( (int16_t) frame.data[7] << 8) 
							+ ( (int16_t) frame.data[6] );

	const float torqueUnit = 0.01;
	const float speedUnit = 1.0;
	const float angleUnit = 1.0;

	int temperature = (int) temperatureParameter;
	float torque = torqueParameter * torqueUnit;
	float speed = speedParameter * speedUnit;
	float angle = angleParameter * angleUnit;

	// Adjust to our custom coordinates and units
	torque = -torque;
	speed = -deg2rad(speed);

	// Write the read data into the corresponding place
	scoped_lock lock(m_mutex);
	m_motors[idx]->temperature = temperature;
	m_motors[idx]->torque = torque;
	m_motors[idx]->angle = angle;
	m_motors[idx]->speed = speed;

	// Set update flag
	m_motors[idx]->f_temperature = 1;
	m_motors[idx]->f_torque = 1;
	m_motors[idx]->f_angle = 1;
	m_motors[idx]->f_speed = 1;
}