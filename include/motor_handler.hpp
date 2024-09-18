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

    // Mode command
    void writeTorque(std::vector<int> ids, std::vector<float> torques);
    void writeTorque(std::vector<float> torques);
    void getTorqueFbck(std::vector<int> ids, std::vector<float>& torqueFbck);
    void getTorqueFbck(std::vector<float>& torqueFbck);
    void writeSpeed(std::vector<float> speeds);
    void writeSpeed(std::vector<int> ids, std::vector<float> speeds);
    void getSpeedFbck(std::vector<float>& speedFbck);
    void getSpeedFbck(std::vector<int> ids, std::vector<float>& speedFbck);


    int getModel(int i);

private:
    Listener* m_listener = nullptr;
    Writer* m_writer = nullptr;

    std::vector<int> m_ids;
    std::vector<Motor*> m_motors;
    int m_nbrMotors;

    int openSocket(const char* can_bus);
    

};

#endif