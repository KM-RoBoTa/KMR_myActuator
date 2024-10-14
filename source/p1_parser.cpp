/**
 ****************************************************************************
 * @file        p1_parser.cpp
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

#include "p1_parser.hpp"

using namespace std;

extern vector<Model> p1_models;
extern vector<Model> p2_models;

/**
 * @brief       
 */
P1Parser::P1Parser(vector<Motor*> motors, vector<int> ids, mutex* mutex)
{
    m_mutex = mutex;
	m_motors = motors;
	m_nbrMotors = motors.size();
	m_ids = ids;
}

void P1Parser::parseFrame(can_frame frame)
{
    if (frame.can_id >= 0x501 && frame.can_id <= 0x520) // motion mode
        parseHybridCommand(frame);
    else {
        int command = frame.data[0];

        switch (command)
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

        //case 0xB5: 	parseModel(frame);	break;
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
        case 0xA8:	parsePositionIncrCommand_MT(frame); break;

        default:
            cout << "Unknown function" << endl; 
            break;
        }
    }
}

/*
 *****************************************************************************
 *                               Motor infos
 ****************************************************************************/

// --------- PID ----------- //

// Reads currently used PID, so it could have been changed to RAM
void P1Parser::parsePIDFbck(can_frame frame)
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
	scoped_lock lock(*m_mutex);
	m_motors[idx]->pidReport = pidReport;

	// Set update flag
	m_motors[idx]->fr_PID = 1;
}

void P1Parser::parsePID_RAM_write(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Set up confirmation flag
	scoped_lock lock(*m_mutex);
	m_motors[idx]->fw_PID_RAM= 1;	
}

void P1Parser::parsePID_EEPROM_write(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Set up confirmation flag
	scoped_lock lock(*m_mutex);
	m_motors[idx]->fw_PID_EEPROM = 1;	
}

// --------- Acc settings  ----------- //


// Between 100 and 60000 deg/s² (= 1.74 to 1047 rad/s²)
void P1Parser::parseAccSettingsFbck(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's indexề
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
	scoped_lock lock(*m_mutex);
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
		cout << "Error! P1Parser did not recognize the received acc parameter" << endl;
		exit(1); 
		break;
	}
}

void P1Parser::parseAccSettings_write(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	int mode = frame.data[1];

	// Set up confirmation flag
	scoped_lock lock(*m_mutex);
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
		cout << "Error! P1Parser did not recognize the received acc parameter" << endl;
		exit(1); 
		break;
	}
}

// --------- On/off  ----------- //


void P1Parser::parseShutdown(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Set up confirmation flag
	scoped_lock lock(*m_mutex);
	m_motors[idx]->f_shutdown = 1;		
}

void P1Parser::parseStop(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Set up confirmation flag
	scoped_lock lock(*m_mutex);
	m_motors[idx]->f_stop = 1;		
}

void P1Parser::parseBrakeRelease(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Set up confirmation flag
	scoped_lock lock(*m_mutex);
	m_motors[idx]->f_brake = 1;	
}

void P1Parser::parseBrakeLock(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Set up confirmation flag
	scoped_lock lock(*m_mutex);
	m_motors[idx]->f_brake = 1;	
}

// --------- Status and errors ----------- //

void P1Parser::parseErrorReport(can_frame frame)
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

	scoped_lock lock(*m_mutex);
	m_motors[idx]->errorReport = errorReport;
	m_motors[idx]->f_errorReport = 1;
}

void P1Parser::parseMotorFbck(can_frame frame)
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
	scoped_lock lock(*m_mutex);
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


void P1Parser::parsePhaseReport(can_frame frame)
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

	scoped_lock lock(*m_mutex);
	m_motors[idx]->phaseReport = phaseReport;
	m_motors[idx]->f_phaseReport = 1;
}

// --------- Commands ----------- //

void P1Parser::parseTorqueCommand(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Set up confirmation flag
	scoped_lock lock(*m_mutex);
	m_motors[idx]->fw_torque = 1;	
}

void P1Parser::parseSpeedCommand(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	// Set up confirmation flag
	scoped_lock lock(*m_mutex);
	m_motors[idx]->fw_speed = 1;			
}

void P1Parser::parseHybridCommand(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x500;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	// Set up confirmation flag
	scoped_lock lock(*m_mutex);
	m_motors[idx]->fw_hybrid = 1;		
}


// --------- Motor info ----------- //

/*
void P1Parser::parseModel(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

    // Get the model and the corresponding protocol
    string model;
    for (int i=1; i<FRAME_LENGTH; i++)
		model += frame.data[i];

    int cnt = count(p1_models.begin(), p1_models.end(), model);
    Protocol protocol = UNDEF_PTC;
    if (cnt > 0) 
        protocol = PROTOCOL_1;
    else {
        cnt = count(p2_models.begin(), p2_models.end(), model);

        if (cnt > 0)
            protocol = PROTOCOL_2;
        else {
            cout << "Could not identify the protocol of motor " << id << ". Exiting" << endl;
            exit(1);
        }
    }

	// Write the read data into the corresponding place
	scoped_lock lock(*m_mutex);
    m_motors[idx]->model.clear(); // Clear the previous string in case it was already called
    m_motors[idx]->model = model;
    m_motors[idx]->protocol = protocol;

	// Set update flag
	m_motors[idx]->f_model = 1;
}*/

void P1Parser::parseOperatingMode(can_frame frame)
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

	scoped_lock lock(*m_mutex);
	m_motors[idx]->operatingMode = mode;
	m_motors[idx]->fr_mode = 1;
}

void P1Parser::parsePowerConsumption(can_frame frame)
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

	scoped_lock lock(*m_mutex);
	m_motors[idx]->power = power;
	m_motors[idx]->fr_power = 1;
}

void P1Parser::parseRuntime(can_frame frame)
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

	scoped_lock lock(*m_mutex);
	m_motors[idx]->runtime = runtime;
	m_motors[idx]->fr_runtime = 1;
}

void P1Parser::parseSoftwareDate(can_frame frame)
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

	scoped_lock lock(*m_mutex);
	m_motors[idx]->softwareDate = date;
	m_motors[idx]->fr_softwareDate = 1;
}


// --------------------- Other settings ------------------- //

void P1Parser::parseCompoundFct(can_frame frame)
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

void P1Parser::parseMultiturnReset(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	scoped_lock lock(*m_mutex);
	m_motors[idx]->fw_multiturnReset = 1;
}

void P1Parser::parseCANFilter(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	scoped_lock lock(*m_mutex);
	m_motors[idx]->fw_CANFilter = 1;
}

void P1Parser::parseActiveError(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	scoped_lock lock(*m_mutex);
	m_motors[idx]->fw_activeErrorFbck = 1;
}

void P1Parser::parseMultiturnMode(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	scoped_lock lock(*m_mutex);
	m_motors[idx]->fw_multiturnMode = 1;
}


// ---- Position feedbacks ---- //

void P1Parser::parseEncoderFbck(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Parse data
	int32_t encoderPosition = ( (int32_t) frame.data[7] << 24) +
							  ( (int32_t) frame.data[6] << 16) +
							  ( (int32_t) frame.data[5] <<  8) +
							  ( (int32_t) frame.data[4]      );

	scoped_lock lock(*m_mutex);
	m_motors[idx]->encoderPosition = encoderPosition;
	m_motors[idx]->fr_encoder = 1;
}

void P1Parser::parseRawEncoderFbck(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Parse data
	uint32_t encoderPosition = ( (uint32_t) frame.data[7] << 24) +
							   ( (uint32_t) frame.data[6] << 16) +
							   ( (uint32_t) frame.data[5] <<  8) +
							   ( (uint32_t) frame.data[4]      );

	scoped_lock lock(*m_mutex);
	m_motors[idx]->encoderRawPosition = encoderPosition;
	m_motors[idx]->fr_encoderRaw = 1;
}

void P1Parser::parseEncoderZeroOffsetRead(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Parse data
	uint32_t encoderPosition = ( (uint32_t) frame.data[7] << 24) +
							   ( (uint32_t) frame.data[6] << 16) +
							   ( (uint32_t) frame.data[5] <<  8) +
							   ( (uint32_t) frame.data[4]      );

	scoped_lock lock(*m_mutex);
	m_motors[idx]->encoderZeroOffset = encoderPosition;
	m_motors[idx]->fr_encoderZeroOffset = 1;
}

void P1Parser::parseEncoderZeroOffsetWrite(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// DEBUG
	uint32_t encoderPosition = ( (uint32_t) frame.data[7] << 24) +
							   ( (uint32_t) frame.data[6] << 16) +
							   ( (uint32_t) frame.data[5] <<  8) +
							   ( (uint32_t) frame.data[4]      );	
	cout << "New zero position: " << (int) encoderPosition << endl;

	// Get the vector's index
	int idx = getIndex(m_ids, id);		

	scoped_lock lock(*m_mutex);
	m_motors[idx]->fw_encoderZeroOffset = 1;
}


void P1Parser::parseEncoderFbck_ST(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	// Parse data
	uint16_t encoderPosition     =  ( (uint16_t) frame.data[3] << 8) +
							        ( (uint16_t) frame.data[2]     );
	uint16_t encoderRawPosition  =  ( (uint16_t) frame.data[5] << 8) +
							        ( (uint16_t) frame.data[4]     );
	uint16_t encoderZeroPosition =  ( (uint16_t) frame.data[7] << 8) +
							        ( (uint16_t) frame.data[6]     );

	scoped_lock lock(*m_mutex);
	m_motors[idx]->encoderPosition_ST = encoderPosition;
	m_motors[idx]->encoderRawPosition_ST = encoderRawPosition;
	m_motors[idx]->encoderZeroOffset_ST = encoderZeroPosition;

	m_motors[idx]->fr_encoder_ST = 1;
	m_motors[idx]->fr_encoderRaw_ST = 1;
	m_motors[idx]->fr_encoderZeroOffset_ST = 1;
}

void P1Parser::parsePositionFbck_MT(can_frame frame)
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

	scoped_lock lock(*m_mutex);
	m_motors[idx]->angle_posFbck_MT = position;
	m_motors[idx]->fr_position_MT = 1;
}


void P1Parser::parsePositionFbck_ST(can_frame frame)
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

	scoped_lock lock(*m_mutex);
	m_motors[idx]->angle_posFbck_ST = position;
	m_motors[idx]->fr_position_ST = 1;
}

void P1Parser::parsePositionCommand_MT(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	scoped_lock lock(*m_mutex);
	m_motors[idx]->fw_position_MT = 1;
}

void P1Parser::parsePositionCommand_ST(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	scoped_lock lock(*m_mutex);
	m_motors[idx]->fw_position_ST = 1;
}

void P1Parser::parsePositionIncrCommand_MT(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);	

	scoped_lock lock(*m_mutex);
	m_motors[idx]->fw_positionIncr_MT = 1;
}