/**
 ****************************************************************************
 * @file        listener.cpp
 * @brief       CAN bus listener, running in own thread
 ****************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 09/2024
 * @authors  kamilo.melo@km-robota.com, 09/2024
 ****************************************************************************
 */

// Standard libraries
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <cmath>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <cstring>
#include <net/if.h> // network devices
#include <sstream> // Convert dec-hex

#include "listener.hpp"


using namespace std;


/**
 * @brief       Start the CAN bus listener
 * @param[in]   s Socket
 */
Listener::Listener(vector<Motor*> motors, vector<int> ids, int s)
{
	m_motors = motors;
	m_nbrMotors = motors.size();
    m_stopThread = false;
    m_thread = thread(&Listener::listenerLoop, this, s);
	m_ids = ids;

    cout << "Creating the CAN listener's thread..." << endl;
	usleep(50000);  
}

/**
 * @brief	Class destructor. Takes care of safely stopping the thread
 */
Listener::~Listener()
{
	// Change the internal boolean to get the thread out of its loop function
	{
		scoped_lock lock(m_mutex);
		m_stopThread = true;	
	}	
    
	// Block the main thread until the Listener thread finishes
    if (m_thread.joinable()) 
        m_thread.join();

	cout << "Listener thread safely stopped" << endl;
}

/**
 * @brief       
 * @param[in]   s Socket
 * @retval      1 when the thread is finished
 */
int Listener::listenerLoop(int s)
{
	bool stopThread = 0;

	// -----  Start of the main loop -----
	cout << "Starting CAN bus monitoring" << endl;
    while(!stopThread) {

        struct can_frame frame;
        int nbytes = read(s, &frame, sizeof(struct can_frame));

        if (nbytes > 0) {
			
            // Parse the received frame

			// TEMPORARY
			int function = frame.data[0];

			switch (function)
			{
			case 0xB5:
				readModel(frame);
				break;
			
			default:
				break;
			}
        }

		// Thread sleep for scheduling
		std::this_thread::sleep_for(chrono::microseconds(50));

		{
			scoped_lock lock(m_mutex);
			stopThread = m_stopThread;	
		}	
    }

    return 0;
}


/*
 *****************************************************************************
 *                               Motor infos
 ****************************************************************************/


void Listener::readModel(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.can_id - 0x240;

	// Get the vector's index
	int idx = getIndex(m_ids, id);

	// Write the read data into the corresponding place
	scoped_lock lock(m_mutex);
	for (int i=1; i<FRAME_LENGTH; i++)
		m_motors[idx]->model[i] = frame.data[i];
}