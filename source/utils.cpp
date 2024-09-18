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

#include "utils.hpp"
#include <cmath>

// Function to return the index of an element k
int getIndex(std::vector<int> v, int k) 
{ 
    auto it = std::find(v.begin(), v.end(), k); 
    int index = -1;
  
    // If element was found 
    if (it != v.end())  
        index = it - v.begin(); 
    return index;
} 

float deg2rad(float deg)
{
    return (deg * M_PI / 180.0);
}

float rad2deg(float rad)
{
    return (rad * 180 / M_PI);
}
