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

#include <sstream> // Convert dec-hex
#include <cstring>

using namespace std;

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


/**
 * @brief   Get current time structure
 * @return  Current time structure
 */
timespec time_s()
{
    struct timespec real_time;

    if (clock_gettime(CLOCK_REALTIME, &real_time) == -1 )
    {
        perror("clock gettime");
        exit( EXIT_FAILURE );
    }

    return real_time;
}

/**
 * @brief       Get elapsed time, in microseconds
 * @param[in]   t2 End time structure (gotten with time_s)
 * @param[in]   t2 Start time structure (gotten with time_s)
 * @return      Elapsed time between t1 and t2, in us
 */
double get_delta_us(struct timespec t2, struct timespec t1)
{
    struct timespec td;
    td.tv_nsec = t2.tv_nsec - t1.tv_nsec;
    td.tv_sec  = t2.tv_sec - t1.tv_sec;
    if (td.tv_sec > 0 && td.tv_nsec < 0)
    {
        td.tv_nsec += 1000000000;
        td.tv_sec--;
    }

    return(td.tv_sec*1000000 + td.tv_nsec/1000);
}

std::string convertToHex(int dec) 
{
    std::stringstream ss;
    ss << std::hex << dec; // int decimal_value
    std::string res ( ss.str() );

    return res;
}