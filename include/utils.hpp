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

#ifndef KMR_MYACTU_UTILS_HPP
#define KMR_MYACTU_UTILS_HPP

#include <vector>
#include <algorithm>
#include <iostream>
#include <unistd.h> // Provides the usleep function


// Function to return the index of an element k
int getIndex(std::vector<int> v, int k);

float deg2rad(float deg);
float rad2deg(float rad);


timespec time_s();
double get_delta_us(struct timespec t2, struct timespec t1);
std::string convertToHex(int dec) ;


template<typename T>
T saturate(T min, T max, T val)
{
    if (val < min)
        return min;
    else if (val > max)
        return max;
    else
        return val;
}

// Result between -b and +b
template<typename T>
T modulo(T a, T b)
{
    if (b == 0) {
        std::cout << "Error! Modulo by 0" << std::endl;
        exit(1);
    }

    int k = (int)( (float)a /(float)b );
    std::cout << "k = " << k << std::endl;
    return (a-k*b);
}


#endif