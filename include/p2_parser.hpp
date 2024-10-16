/**
 ******************************************************************************
 * @file            p2_parser.hpp
 * @brief           Header for the p2_parser.cpp file
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 09/2024
 * @authors  kamilo.melo@km-robota.com, 09/2024
 ******************************************************************************
 */

#ifndef KMR_MYACTU_P2_PARSER_HPP
#define KMR_MYACTU_P2_PARSER_HPP

#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <linux/can.h>

#include "config.hpp"
#include "utils.hpp"


/**
 * @brief   Parser for protocol 2
 */
class P2Parser {
public:
    P2Parser(std::vector<Motor*> motors, std::vector<int> ids, std::mutex* mutex);
    //~P1Parser();

    void parseFrame(can_frame frame); 

private:
    std::mutex* m_mutex;
    std::vector<Motor*> m_motors;
    int m_nbrMotors;
    std::vector<int> m_ids;

    // ---------  ----------- //
    void parseDefaultCommandType(can_frame frame);
    void parseMultiturnMode(can_frame frame);
};

#endif