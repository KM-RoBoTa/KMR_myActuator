/**
 ******************************************************************************
 * @file            listener.hpp
 * @brief           Header for the listener.cpp file
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 09/2024
 * @authors  kamilo.melo@km-robota.com, 09/2024
 ******************************************************************************
 */

#ifndef KMR_MYACTU_LISTENER_HPP
#define KMR_MYACTU_LISTENER_HPP

#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <linux/can.h>

#include "config.hpp"
#include "utils.hpp"


/**
 * @brief   CAN bus listener, running in its own thread
 */
class Listener {
public:
    Listener(std::vector<Motor*> motors, std::vector<int> ids, int s);
    ~Listener();


private:
    // Thread
    bool m_stopThread = 0;
    std::thread m_thread;
    std::mutex m_mutex;

    std::vector<Motor*> m_motors;
    int m_nbrMotors;
    std::vector<int> m_ids;

    int listenerLoop(int s);
    void readModel(can_frame frame);
};

#endif