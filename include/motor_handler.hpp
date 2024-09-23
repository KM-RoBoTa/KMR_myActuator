/**
 ******************************************************************************
 * @file            motor_handler.hpp
 * @brief           Header for the motor_handler.cpp file
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 09/2024
 * @authors  kamilo.melo@km-robota.com, 09/2024
 ******************************************************************************
 */

#include <iostream>
#include <vector>

#include "listener.hpp"
#include "writer.hpp"

#ifndef KMR_MYACTU_MOTOR_HANDLER_HPP
#define KMR_MYACTU_MOTOR_HANDLER_HPP

/**
 * @brief   CAN bus listener, running in its own thread
 */
class MotorHandler {
public:
    MotorHandler(std::vector<int> ids, const char* can_bus);
    ~MotorHandler();

    void pingMotors();

    // --------- PID ----------- //
    bool getPID(std::vector<int> ids, std::vector<PIDReport>& pidReports);
    bool getPID(std::vector<PIDReport>& pidReports);
    bool writePID_RAM(std::vector<int> ids, std::vector<PIDReport> pidReports);
    bool writePID_RAM(std::vector<PIDReport> pidReports);
    bool writePID_EEPROM(std::vector<int> ids, std::vector<PIDReport> pidReports);
    bool writePID_EEPROM(std::vector<PIDReport> pidReports);

    // --------- Acc settings  ----------- //
    bool getAccelerationSettings(std::vector<int> ids, ACC_SETTINGS setting, std::vector<int>& accs);
    bool getAccelerationSettings(ACC_SETTINGS setting, std::vector<int>& accs);
    bool writeAccelerationSettings(std::vector<int> ids, ACC_SETTINGS setting, std::vector<int> accs);
    bool writeAccelerationSettings(ACC_SETTINGS setting, std::vector<int> accs);

    // --------- On/off  ----------- //
    bool shutdownMotors(std::vector<int> ids);
    bool shutdownMotors();
    bool stopMotors(std::vector<int> ids);
    bool stopMotors();
    bool resetMotors(std::vector<int> ids);
    bool resetMotors();
    bool releaseBrake(std::vector<int> ids);      
    bool releaseBrake();    
    bool lockBrake(std::vector<int> ids);     
    bool lockBrake();  

    // --------- Status and errors  ----------- //
    bool getErrorReport(std::vector<int> ids, std::vector<ErrorReport>& errorReports);
    bool getErrorReport(std::vector<ErrorReport>& errorReports);
    bool getPhaseReport(std::vector<int> ids, std::vector<PhaseReport>& phaseReports);
    bool getPhaseReport(std::vector<PhaseReport>& phaseReports);

    bool getTorqueFbck(std::vector<int> ids, std::vector<float>& torqueFbck);
    bool getTorqueFbck(std::vector<float>& torqueFbck);
    bool getSpeedFbck(std::vector<int> ids, std::vector<float>& speedFbck);
    bool getSpeedFbck(std::vector<float>& speedFbck);
    //bool getAngleFbck(std::vector<int> ids, std::vector<float>& speedFbck);
    //bool getAngleFbck(std::vector<float>& speedFbck);
    bool getTemperatureFbck(std::vector<int> ids, std::vector<int>& tempFbck);
    bool getTemperatureFbck(std::vector<int>& tempFbck);
    bool getFullFbck(std::vector<int> ids, std::vector<StatusReport>& statusReport);
    bool getFullFbck(std::vector<StatusReport>& statusReport);

    // ----------  Commands ----------- //
    bool writeTorque(std::vector<int> ids, std::vector<float> torques);
    bool writeTorque(std::vector<float> torques);
    bool writeSpeed(std::vector<int> ids, std::vector<float> speeds);
    bool writeSpeed(std::vector<float> speeds);
    bool writeMotion(std::vector<int> ids, std::vector<float> pos, std::vector<float> speeds,
                    std::vector<float> Kps, std::vector<float> Kds, std::vector<float> Tff);
    bool writeMotion(std::vector<float> pos, std::vector<float> speeds,
                    std::vector<float> Kps, std::vector<float> Kds, std::vector<float> Tff);

    // ----------  Motor info ----------- //
    bool getModel(std::vector<int> ids, std::vector<std::string>& models);
    bool getModel(std::vector<std::string>& models);
    bool getOperatingMode(std::vector<int> ids, std::vector<OperatingMode>& modes); 
    bool getOperatingMode(std::vector<OperatingMode>& modes);
    bool getPowerConsumption(std::vector<int> ids, std::vector<float>& powers);
    bool getPowerConsumption(std::vector<float>& powers);
    bool getRuntime(std::vector<int> ids, std::vector<float>& runtimes);
    bool getRuntime(std::vector<float>& runtimes);   
    bool getSoftwareDate(std::vector<int> ids, std::vector<int>& dates);
    bool getSoftwareDate(std::vector<int>& dates);   

    // ----------  Other settings ----------- //
    bool enableCANFilter(std::vector<int> ids);
    bool enableCANFilter();
    bool disableCANFilter(std::vector<int> ids);
    bool disableCANFilter();

    bool resetMultiturnCounter(std::vector<int> ids);
    bool resetMultiturnCounter();

    bool enableActiveErrorFbck(std::vector<int> ids);
    bool enableActiveErrorFbck();
    bool disableActiveErrorFbck(std::vector<int> ids);
    bool disableActiveErrorFbck();

    bool setMultiturnMode(std::vector<int> ids);
    bool setMultiturnMode();
    bool setSingleturnMode(std::vector<int> ids);
    bool setSingleturnMode();


private:
    Listener* m_listener = nullptr;
    Writer* m_writer = nullptr;

    std::vector<int> m_ids;
    std::vector<Motor*> m_motors;
    int m_nbrMotors;

    int openSocket(const char* can_bus);
    

};

#endif