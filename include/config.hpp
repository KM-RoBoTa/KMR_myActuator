/**
 ******************************************************************************
 * @file            writer.hpp
 * @brief           Header for the writer.cpp file
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 09/2024
 * @authors  kamilo.melo@km-robota.com, 09/2024
 ******************************************************************************
 */

#ifndef KMR_MYACTU_CONFIG_HPP
#define KMR_MYACTU_CONFIG_HPP

#define FRAME_LENGTH    8

/**
 * @brief   Structure saving the info of a data field
 */
struct Motor {
    int id;
    char model[FRAME_LENGTH-1];

    // Constructor
    Motor(int id)
    {
        this->id = id;
        std::cout << "created " << std::endl;
    }
};

#endif