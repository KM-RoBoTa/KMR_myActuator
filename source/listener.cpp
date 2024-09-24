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

			case 0x80:	parseShutdown(frame); break;
			case 0x81:	parseStop(frame); break;
			case 0x77:	parseBrakeRelease(frame); break;
			case 0x78:	parseBrakeLock(frame); break;

			case 0x9A:	parseErrorReport(frame); break;
			case 0x9C:	parseMotorFbck(frame); break;
			case 0x9D:	parsePhaseReport(frame); break;

			case 0xA1:	parseTorqueCommand(frame); break;
			case 0xA2: 	parseSpeedCommand(frame); break;
			case 0x500:	parseMotionModeCommand(frame); break;

			case 0xB5: 	parseModel(frame);	break;
			case 0x70:	parseOperatingMode(frame); break;
			case 0x71:	parsePowerConsumption(frame); break;
			case 0xB1:	parseRuntime(frame); break;
			case 0xB2:	parseSoftwareDate(frame); break;

			case 0x20: 	parseCompoundFct(frame); break;

			case 0x60:	parseEncoderFbck(frame); break;
			case 0x61:	parseRawEncoderFbck(frame); break;
			case 0x62:	parseEncoderZeroOffsetRead(frame); break;
			case 0x63:	parseEncoderZeroOffsetWrite(frame); break; // input value
			case 0x64:	parseEncoderZeroOffsetWrite(frame); break; // current position

			case 0x90:	parseEncoderFbck_ST(frame); break;

			case 0x92:	parsePositionFbck_MT(frame); break;
			case 0x94:	parsePositionFbck_ST(frame); break;

			case 0xA4:	parsePositionCommand_MT(frame); break;
			case 0xA6:	parsePositionCommand_ST(frame); break;

			default:
				if (frame.can_id == 0x501) 
					parseMotionModeCommand(frame);
				else
					cout << "Unknown function" << endl; 
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

bool Listener::motion_written(int id)
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
		available = m_motors[idx]->fw_motion;

		// Clear update flag
		m_motors[idx]->fw_motion = 0;
	}

	return available;	
}

// --------- Motor info ----------- //

bool Listener::getModel(int id, string& model)
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

		// Clear update flag and the saved string in case the getModel function gets recalled
		m_motors[idx]->f_model = 0;
		m_motors[idx]->model.clear();
	}

	return available;
}


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

bool Listener::getEncoderPosition(int id, int& position)
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

bool Listener::getRawEncoderPosition(int id, int& position)
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

bool Listener::getEncoderZeroOffset(int id, int& position)
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

	PIDReport pidReport;
	pidReport.Kp_torque = frame.data[2];
	pidReport.Ki_torque = frame.data[3];
	pidReport.Kp_speed = frame.data[4];
	pidReport.Ki_speed = frame.data[5];
	pidReport.Kp_pos = frame.data[6];
	pidReport.Ki_pos = frame.data[7];

	// Write the read data into the corresponding place
	scoped_lock lock(m_mutex);
	m_motors[idx]->pidReport = pidReport;

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

// --------- On/off  ----------- //


void Listener::parseShutdown(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Set up confirmation flag
	scoped_lock lock(m_mutex);
	m_motors[idx]->f_shutdown = 1;		
}

void Listener::parseStop(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Set up confirmation flag
	scoped_lock lock(m_mutex);
	m_motors[idx]->f_stop = 1;		
}

void Listener::parseBrakeRelease(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Set up confirmation flag
	scoped_lock lock(m_mutex);
	m_motors[idx]->f_brake = 1;	
}

void Listener::parseBrakeLock(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Set up confirmation flag
	scoped_lock lock(m_mutex);
	m_motors[idx]->f_brake = 1;	
}

// --------- Status and errors ----------- //

void Listener::parseErrorReport(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Analyze the data in the frame
	int8_t temperatureParameter = frame.data[1];
	int8_t brakeReleaseParameter = frame.data[3];
	int16_t voltageParameter = ( (int16_t) frame.data[5] << 8) +
							   ( (int16_t) frame.data[4] );
	int16_t errorParameter = ( (int16_t) frame.data[7] << 8) +
							 ( (int16_t) frame.data[6] );

	const float unitsVoltage = 0.1;
	ErrorReport errorReport;

	errorReport.temperature = (int) temperatureParameter;
	errorReport.brakeReleased = (int) brakeReleaseParameter;  // boolean-type
	errorReport.voltage = voltageParameter * unitsVoltage;

	// Parse the error
	if ( (errorParameter & ERR_MOTOR_STALL) != 0)
		errorReport.err_motorStall = 1;
	if ( (errorParameter & ERR_UNDERVOLTAGE) != 0)
		errorReport.err_undervoltage = 1;
	if ( (errorParameter & ERR_OVERVOLTAGE) != 0)
		errorReport.err_overvoltage = 1;
	if ( (errorParameter & ERR_OVERCURRENT) != 0)
		errorReport.err_overcurrent = 1;
	if ( (errorParameter & ERR_POWER_OVERRUN) != 0)
		errorReport.err_powerOverrun = 1;
	if ( (errorParameter & ERR_CALIB_PARAM) != 0)
		errorReport.err_calibParameter = 1;
	if ( (errorParameter & ERR_SPEEDING) != 0)
		errorReport.err_speeding = 1;
	if ( (errorParameter & ERR_OVERHEATING) != 0)
		errorReport.err_overheating = 1;
	if ( (errorParameter & ERR_ENCODER_CALIB) != 0)
		errorReport.err_encoderCalib = 1;

	scoped_lock lock(m_mutex);
	m_motors[idx]->errorReport = errorReport;
	m_motors[idx]->f_errorReport = 1;
}

void Listener::parseMotorFbck(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	// Convert from parameters to SI values
	int8_t temperatureParameter = frame.data[1];
	int16_t torqueParameter = ((int16_t) frame.data[3] << 8) +
							  ((int16_t) frame.data[2] );
	int16_t speedParameter  = ((int16_t) frame.data[5] << 8) +
							  ((int16_t) frame.data[4] );
	int16_t angleParameter  = ((int16_t) frame.data[7] << 8) +
							  ((int16_t) frame.data[6] );

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
	angle = -deg2rad(angle);

	// Write the read data into the corresponding place
	scoped_lock lock(m_mutex);
	m_motors[idx]->temperature = temperature;
	m_motors[idx]->torque = torque;
	m_motors[idx]->angle = angle;
	m_motors[idx]->speed = speed;

	// Set update flag
	m_motors[idx]->fr_temperature = 1;
	m_motors[idx]->fr_torque = 1;
	m_motors[idx]->fr_angle = 1;
	m_motors[idx]->fr_speed = 1;
	m_motors[idx]->fr_fullFbck = 1;
}


void Listener::parsePhaseReport(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Analyze data
	int8_t temperatureParameter = frame.data[1];
	int16_t phaseAParameter = ( (int16_t) frame.data[3] << 8) +
                              ( (int16_t) frame.data[2] );
	int16_t phaseBParameter = ( (int16_t) frame.data[5] << 8) +
                              ( (int16_t) frame.data[4] );
	int16_t phaseCParameter = ( (int16_t) frame.data[7] << 8) +
                              ( (int16_t) frame.data[6] );

	const float unitsPhase = 0.01;
	PhaseReport phaseReport;
	phaseReport.temperature = (int) temperatureParameter;
	phaseReport.currentPhaseA = phaseAParameter * unitsPhase;
	phaseReport.currentPhaseB = phaseBParameter * unitsPhase;
	phaseReport.currentPhaseC = phaseCParameter * unitsPhase;

	scoped_lock lock(m_mutex);
	m_motors[idx]->phaseReport = phaseReport;
	m_motors[idx]->f_phaseReport = 1;
}

// --------- Commands ----------- //

void Listener::parseTorqueCommand(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Set up confirmation flag
	scoped_lock lock(m_mutex);
	m_motors[idx]->fw_torque = 1;	
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

void Listener::parseMotionModeCommand(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x500;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	// Set up confirmation flag
	scoped_lock lock(m_mutex);
	m_motors[idx]->fw_motion = 1;		
}


// --------- Motor info ----------- //

void Listener::parseModel(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	// Write the read data into the corresponding place
	scoped_lock lock(m_mutex);
	for (int i=1; i<FRAME_LENGTH; i++)
		m_motors[idx]->model += frame.data[i];

	// Set update flag
	m_motors[idx]->f_model = 1;
}

void Listener::parseOperatingMode(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	// Parse
	OperatingMode mode;
	switch (frame.data[7])
	{
	case 0x01:
		mode = TORQUE_LOOP;
		break;
	case 0x02:
		mode = SPEED_LOOP;
		break;
	case 0x03:
		mode = POSITION_LOOP;
		break;
	default:
		cout << "Operating mode unknown! Exiting" << endl;
		exit(1);
		break;
	}

	scoped_lock lock(m_mutex);
	m_motors[idx]->operatingMode = mode;
	m_motors[idx]->fr_mode = 1;
}

void Listener::parsePowerConsumption(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	// Parse
	const float unitsPower = 0.1;
	int16_t powerParameter = ( (int16_t) frame.data[7] << 8) +
							 ( (int16_t) frame.data[6] );

	float power = powerParameter * unitsPower;

	scoped_lock lock(m_mutex);
	m_motors[idx]->power = power;
	m_motors[idx]->fr_power = 1;
}

void Listener::parseRuntime(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	// Parse
	int32_t runtimeParameter = ( (int32_t) frame.data[7] << 24) +
                               ( (int32_t) frame.data[6] << 16) +
                               ( (int32_t) frame.data[5] <<  8) +
                               ( (int32_t) frame.data[4]      );
	int runtime_ms = (int) runtimeParameter;  // ms
	float runtime = (float)runtime_ms / 1000.0;

	scoped_lock lock(m_mutex);
	m_motors[idx]->runtime = runtime;
	m_motors[idx]->fr_runtime = 1;
}

void Listener::parseSoftwareDate(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	// Parse
	int32_t dateParameter = ( (int32_t) frame.data[7] << 24) +
							( (int32_t) frame.data[6] << 16) +
							( (int32_t) frame.data[5] <<  8) +
							( (int32_t) frame.data[4]      );
	int date = (int) dateParameter;

	scoped_lock lock(m_mutex);
	m_motors[idx]->softwareDate = date;
	m_motors[idx]->fr_softwareDate = 1;
}


// --------------------- Other settings ------------------- //

void Listener::parseCompoundFct(can_frame frame)
{
	switch (frame.data[1])
	{
	case 0x01:
		parseMultiturnReset(frame);
		break;
	case 0x02:
		parseCANFilter(frame);
		break;
	case 0x03:
		parseActiveError(frame);
		break;
	case 0x04:
		parseMultiturnMode(frame);
		break;
	default:
		cout << "Error! Unknown function in compound 0x20. Exiting" << endl;
		exit(1);
		break;
	}
}

void Listener::parseMultiturnReset(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	scoped_lock lock(m_mutex);
	m_motors[idx]->fw_multiturnReset = 1;
}

void Listener::parseCANFilter(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	scoped_lock lock(m_mutex);
	m_motors[idx]->fw_CANFilter = 1;
}

void Listener::parseActiveError(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	scoped_lock lock(m_mutex);
	m_motors[idx]->fw_activeErrorFbck = 1;
}

void Listener::parseMultiturnMode(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	scoped_lock lock(m_mutex);
	m_motors[idx]->fw_multiturnMode = 1;
}


// ---- Position feedbacks ---- //

void Listener::parseEncoderFbck(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Parse data
	int64_t encoderParameter = ( (int64_t) frame.data[7] << 24) +
							   ( (int64_t) frame.data[6] << 16) +
							   ( (int64_t) frame.data[5] <<  8) +
							   ( (int64_t) frame.data[4]      );
	int encoderPosition = (int) encoderParameter;

	scoped_lock lock(m_mutex);
	m_motors[idx]->encoderPosition = encoderPosition;
	m_motors[idx]->fr_encoder = 1;
}

void Listener::parseRawEncoderFbck(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Parse data
	int64_t encoderParameter = ( (int64_t) frame.data[7] << 24) +
							   ( (int64_t) frame.data[6] << 16) +
							   ( (int64_t) frame.data[5] <<  8) +
							   ( (int64_t) frame.data[4]      );
	int encoderPosition = (int) encoderParameter;

	scoped_lock lock(m_mutex);
	m_motors[idx]->encoderRawPosition = encoderPosition;
	m_motors[idx]->fr_encoderRaw = 1;
}

void Listener::parseEncoderZeroOffsetRead(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Parse data
	int64_t encoderParameter = ( (int64_t) frame.data[7] << 24) +
							   ( (int64_t) frame.data[6] << 16) +
							   ( (int64_t) frame.data[5] <<  8) +
							   ( (int64_t) frame.data[4]      );
	int encoderPosition = (int) encoderParameter;

	scoped_lock lock(m_mutex);
	m_motors[idx]->encoderZeroOffset = encoderPosition;
	m_motors[idx]->fr_encoderZeroOffset = 1;
}

void Listener::parseEncoderZeroOffsetWrite(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);		

	scoped_lock lock(m_mutex);
	m_motors[idx]->fw_encoderZeroOffset = 1;
}


void Listener::parseEncoderFbck_ST(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Parse data
	int16_t encoderParameter     = ( (int16_t) frame.data[3] << 8) +
							       ( (int16_t) frame.data[2]     );
	int16_t encoderRawParameter  = ( (int16_t) frame.data[5] << 8) +
							       ( (int16_t) frame.data[4]     );
	int16_t encoderZeroParameter = ( (int16_t) frame.data[7] << 8) +
							       ( (int16_t) frame.data[6]     );
	int encoderPosition = (int) encoderParameter;
	int encoderRawPosition_ST = (int) encoderRawParameter;
	int encoderZeroOffset_ST = (int) encoderZeroParameter;

	scoped_lock lock(m_mutex);
	m_motors[idx]->encoderPosition_ST = encoderPosition;
	m_motors[idx]->encoderRawPosition_ST = encoderRawPosition_ST;
	m_motors[idx]->encoderZeroOffset_ST = encoderZeroOffset_ST;

	m_motors[idx]->fr_encoder_ST = 1;
	m_motors[idx]->fr_encoderRaw_ST = 1;
	m_motors[idx]->fr_encoderZeroOffset_ST = 1;
}

void Listener::parsePositionFbck_MT(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Parse data
	int32_t positionParameter = ( (int32_t) frame.data[7] << 24) +
							    ( (int32_t) frame.data[6] << 16) +
							    ( (int32_t) frame.data[5] <<  8) +
							    ( (int32_t) frame.data[4]      );

	const float unitsPosition = 0.01;
	float position = (float) positionParameter * unitsPosition;

	// Convert here
	position = -deg2rad(position);

	scoped_lock lock(m_mutex);
	m_motors[idx]->angle_posFbck_MT = position;
	m_motors[idx]->fr_position_MT = 1;
}


void Listener::parsePositionFbck_ST(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Parse data
	int16_t positionParameter = ( (int16_t) frame.data[7] << 8) +
							    ( (int16_t) frame.data[6]     );

	const float unitsPosition = 0.01;
	float position = (float) positionParameter * unitsPosition;

	// Convert here
	position = -deg2rad(position);

	if (position > M_PI)
		position = position - 2*M_PI;
	else if (position < -M_PI)
		position = position + 2*M_PI;

	scoped_lock lock(m_mutex);
	m_motors[idx]->angle_posFbck_ST = position;
	m_motors[idx]->fr_position_ST = 1;
}

void Listener::parsePositionCommand_MT(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	scoped_lock lock(m_mutex);
	m_motors[idx]->fw_position_MT = 1;
}

void Listener::parsePositionCommand_ST(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	scoped_lock lock(m_mutex);
	m_motors[idx]->fw_position_ST = 1;
}