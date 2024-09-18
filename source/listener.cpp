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
			case 0xB5: 	readModel(frame);	break;
			case 0xA1:	parseTorqueCommand(frame); break;
			case 0x9C:	parseMotorFbck(frame); break;
			default:
				break;
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