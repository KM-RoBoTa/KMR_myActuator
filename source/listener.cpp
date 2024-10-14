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

vector<Model> p1_models = {X6_7, X6_40, X8_25, X6_8, X8_20, X8_60, X10_40, X10_100};
vector<Model> p2_models = {X4_24, X8_90, X12_150, X15_400};


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

	// Create the protocol-specific parsers
	p1Parser = new P1Parser(motors, ids, &m_mutex);
	p2Parser = new P2Parser(motors, ids, &m_mutex);

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
			Protocol protocol = getProtocol(frame);
			
			if (protocol == PROTOCOL_1)
				p1Parser->parseFrame(frame);
			else if (protocol == PROTOCOL_2)
				p2Parser->parseFrame(frame);
			else
				cout << "Error! Unknown packet" << endl;
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


Protocol Listener::getProtocol(can_frame frame)
{
	Protocol protocol = UNDEF_PTC;

	if ( (frame.can_id >= 0x241 && frame.can_id <= 0x260) ||
		 (frame.can_id >= 0x501 && frame.can_id <= 0x520) )
		protocol = PROTOCOL_1;
	else
		protocol = PROTOCOL_2;

	return protocol;
}


/*
 *****************************************************************************
 *                             Interface with main thread
 ****************************************************************************/

// --------- PID ----------- //

bool Listener::getPID(int id, PIDReport& pidReport)
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
		pidReport = m_motors[idx]->pidReport;

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


// --------- On/off  ----------- //

bool Listener::shutdown_received(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Shutdown acknowledgement of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->f_shutdown;

		// Clear update flag
		m_motors[idx]->f_shutdown = 0;
	}

	return available;		
}

bool Listener::stop_received(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Stop acknowledgement of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->f_stop;

		// Clear update flag
		m_motors[idx]->f_stop = 0;
	}

	return available;		
}

bool Listener::brake_release_received(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Brake release acknowledgement of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->f_brake;

		// Clear update flag
		m_motors[idx]->f_brake = 0;
	}

	return available;	
}

bool Listener::brake_lock_received(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Brake lock acknowledgement of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->f_brake;

		// Clear update flag
		m_motors[idx]->f_brake = 0;
	}

	return available;	
}


// --------- Status and errors ----------- //

bool Listener::getErrorReport(int id, ErrorReport& errorReport)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;	

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] The error report of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->f_errorReport;
		errorReport = m_motors[idx]->errorReport;

		// Clear update flag
		m_motors[idx]->f_errorReport = 0;
	}

	return available;	
}

bool Listener::getPhaseReport(int id, PhaseReport& phaseReport)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;	

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] The phase report of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->f_phaseReport;
		phaseReport = m_motors[idx]->phaseReport;

		// Clear update flag
		m_motors[idx]->f_phaseReport = 0;
	}

	return available;	
}

bool Listener::getTorque(int id, float& torque)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;	

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] The torque feedback of motor " << id << " timed out!" << endl;
			break;
		}

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_torque;
		torque = m_motors[idx]->torque;

		// Clear update flag
		m_motors[idx]->fr_torque = 0;
	}

	return available;
}

bool Listener::getSpeed(int id, float& speed)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;	

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] The speed feedback of motor " << id << " timed out!" << endl;
			break;
		}

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_speed;
		speed = m_motors[idx]->speed;

		// Clear update flag
		m_motors[idx]->fr_speed = 0;
	}

	return available;
}

bool Listener::getAngle(int id, float& angle)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;	

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] The angle feedback of motor " << id << " timed out!" << endl;
			break;
		}

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_angle;
		angle = m_motors[idx]->angle;

		// Clear update flag
		m_motors[idx]->fr_angle = 0;
	}

	return available;
}

bool Listener::getTemperature(int id, int& temperature)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;	

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] The temperature feedback of motor " << id << " timed out!" << endl;
			break;
		}

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_temperature;
		temperature = m_motors[idx]->temperature;

		// Clear update flag
		m_motors[idx]->fr_temperature = 0;
	}

	return available;
}


bool Listener::getFullStatusReport(int id, StatusReport& statusReport)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;	

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] The full status feedback of motor " << id << " timed out!" << endl;
			break;
		}

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_fullFbck;
		statusReport.angle = m_motors[idx]->angle;
		statusReport.torque = m_motors[idx]->torque;
		statusReport.speed = m_motors[idx]->speed;
		statusReport.temperature = m_motors[idx]->temperature;

		// Clear update flag
		m_motors[idx]->fr_fullFbck = 0;
	}

	return available;
}


// --------- Commands ----------- //

bool Listener::torque_command_received(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Torque command acknowledgement of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fw_torque;

		// Clear update flag
		m_motors[idx]->fw_torque = 0;
	}

	return available;		
}

bool Listener::speedWritten(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Speed command acknowledgement of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fw_speed;

		// Clear update flag
		m_motors[idx]->fw_speed = 0;
	}

	return available;		
}

bool Listener::hybrid_written(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Motion command acknowledgement of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fw_hybrid;

		// Clear update flag
		m_motors[idx]->fw_hybrid = 0;
	}

	return available;	
}

// --------- Motor info ----------- //

/*bool Listener::getModel(int id, string& model)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting the model of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->f_model;
		model = m_motors[idx]->model;

		// Clear update flag
		m_motors[idx]->f_model = 0;
	}

	return available;
}*/


bool Listener::getOperatingMode(int id, OperatingMode& mode)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting the operating mode of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_mode;
		mode = m_motors[idx]->operatingMode;

		// Clear update flag and the saved string in case the getModel function gets recalled
		m_motors[idx]->fr_mode = 0;
	}

	return available;	
}


bool Listener::getPowerConsumption(int id, float& power)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting the power consumption of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_power;
		power = m_motors[idx]->power;

		// Clear update flag and the saved string in case the getModel function gets recalled
		m_motors[idx]->fr_power = 0;
	}

	return available;	
}

bool Listener::getRuntime(int id, float& runtime)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting the runtime of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_runtime;
		runtime = m_motors[idx]->runtime;

		// Clear update flag and the saved string in case the getModel function gets recalled
		m_motors[idx]->fr_runtime = 0;
	}

	return available;	
}

bool Listener::getSoftwareDate(int id, int& date)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting the software date of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_softwareDate;
		date = m_motors[idx]->softwareDate;

		// Clear update flag and the saved string in case the getModel function gets recalled
		m_motors[idx]->fr_softwareDate = 0;
	}

	return available;	
}

// ---- Other settings ---- //


bool Listener::canFilterWritten(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] CAN filter acknowledgement of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fw_CANFilter;

		// Clear update flag
		m_motors[idx]->fw_CANFilter = 0;
	}

	return available;		
}


bool Listener::multiturnResetWritten(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Multiturn reset acknowledgement of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fw_multiturnReset;

		// Clear update flag
		m_motors[idx]->fw_multiturnReset = 0;
	}

	return available;		
}

bool Listener::activeErrorFbckWritten(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Active error status acknowledgement of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fw_activeErrorFbck;

		// Clear update flag
		m_motors[idx]->fw_activeErrorFbck = 0;
	}

	return available;		
}

/// DONE
bool Listener::multiturnModeWritten(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Multiturn mode change acknowledgement of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fw_multiturnMode;

		// Clear update flag
		m_motors[idx]->fw_multiturnMode = 0;
	}

	return available;		
}

bool Listener::getEncoderPosition(int id, int32_t& position)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting the encoder position of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_encoder;
		position = m_motors[idx]->encoderPosition;

		// Clear update flag
		m_motors[idx]->fr_encoder = 0;
	}

	return available;	
}

bool Listener::getRawEncoderPosition(int id, uint32_t& position)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting the raw encoder position of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_encoderRaw;
		position = m_motors[idx]->encoderRawPosition;

		// Clear update flag
		m_motors[idx]->fr_encoderRaw = 0;
	}

	return available;	
}

bool Listener::getEncoderZeroOffset(int id, uint32_t& position)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting the encoder zero offset of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_encoderZeroOffset;
		position = m_motors[idx]->encoderZeroOffset;

		// Clear update flag
		m_motors[idx]->fr_encoderZeroOffset = 0;
	}

	return available;	
}

bool Listener::encoderZeroOffsetWritten(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting encoder's zero writing acknowledgement of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fw_encoderZeroOffset;

		// Clear update flag
		m_motors[idx]->fw_encoderZeroOffset = 0;
	}

	return available;	
}

bool Listener::getEncoderFbck_ST(int id, Encoder_ST& encoder_ST)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting ST encoder's fbck of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_encoderZeroOffset_ST;

		encoder_ST.encoderPosition = m_motors[idx]->encoderPosition_ST;
		encoder_ST.encoderRawPosition = m_motors[idx]->encoderRawPosition_ST;
		encoder_ST.encoderZeroOffset = m_motors[idx]->encoderZeroOffset_ST;

		// Clear update flag
		m_motors[idx]->fr_encoderZeroOffset_ST = 0;
		m_motors[idx]->fr_encoder_ST = 0;
		m_motors[idx]->fr_encoderRaw_ST = 0;
	}

	return available;	
}

bool Listener::getPosition_MT(int id, float& angle)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting multiturn position of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_position_MT;
		angle = m_motors[idx]->angle_posFbck_MT;

		// Clear update flag
		m_motors[idx]->fr_position_MT = 0;

	}

	return available;	
}

bool Listener::getPosition_ST(int id, float& angle)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting singleturn position of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fr_position_ST;
		angle = m_motors[idx]->angle_posFbck_ST;

		// Clear update flag
		m_motors[idx]->fr_position_ST = 0;

	}

	return available;	
}

bool Listener::positionMT_written(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting position writing acknowledgement of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fw_position_MT;

		// Clear update flag
		m_motors[idx]->fw_position_MT = 0;
	}

	return available;	
}

bool Listener::positionST_written(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting position writing acknowledgement of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fw_position_ST;

		// Clear update flag
		m_motors[idx]->fw_position_ST = 0;
	}

	return available;	
}

bool Listener::positionIncrMT_written(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Getting position increment writing acknowledgement of motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fw_positionIncr_MT;

		// Clear update flag
		m_motors[idx]->fw_positionIncr_MT = 0;
	}

	return available;	
}



/*
 *****************************************************************************
 *                           PROTOCOL 2
 ****************************************************************************/

/// DONE
bool Listener::defaultCommandType_written(int id)
{
	int idx = getIndex(m_ids, id);
	Protocol protocol = m_motors[idx]->protocol;
	if (protocol == PROTOCOL_1)
		return 1;

	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Writing of default command type to motor " << id << " timed out!" << endl;
			break;
		}		

		scoped_lock lock(m_mutex);
		available = m_motors[idx]->fw_commandType;

		// Clear update flag
		m_motors[idx]->fw_commandType = 0;
	}

	return available;		
}







