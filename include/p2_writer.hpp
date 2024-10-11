/**
 ******************************************************************************
 * @file            p2_writer.hpp
 * @brief           Header for the p2_writer.cpp file
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 09/2024
 * @authors  kamilo.melo@km-robota.com, 09/2024
 ******************************************************************************
 */

#ifndef KMR_MYACTU_P2_WRITER_HPP
#define KMR_MYACTU_P2_WRITER_HPP

#include <vector>

#include "config.hpp"
#include "utils.hpp"

/**
 * @brief   CAN bus writer for protocol 2 motors
 */
class P2Writer {
public:
    P2Writer(std::vector<Motor*> motors, std::vector<int> ids, int s);
    ~P2Writer();

    int setDefaultCommandType(int id);
    int setMultiturnMode(int id);

private:
    int m_s; // Socket
    std::vector<int> m_ids;

    std::vector<Motor*> m_motors;
    int m_nbrMotors;

};


#endif