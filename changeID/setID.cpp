// Standard libraries
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <cmath>

#include <sys/socket.h>
#include <linux/can.h>
#include <sys/ioctl.h>
#include <cstring>
#include <net/if.h> // network devices
#include <vector>
#include <algorithm>
#include <cstdint>

#include <sstream> // Convert dec-hex

#define RESPONSE_TIMEOUT    1*1000*1000 // us

struct sockaddr_can addr;
struct ifreq ifr;
int s;

// sudo arp-scan --localnet
// sudo ip link set can0 up type can bitrate 1000000
// sshfs user@11.0.0.25:/home ~/sshfs

using namespace std;

enum Model {
    X6_7 = 0, 
    X6_40 = 1,
    X8_25 = 2, 
    X6_8 = 3,
    X8_20 = 4,
    X8_60 = 5,
    X10_40 = 6,
    X10_100 = 7,
    X4_24 = 8, 
    X8_90 = 9, 
    X12_150 = 10,
    X15_400 = 11,
    UNDEF_MODEL
};

enum Protocol {
    PROTOCOL_1 = 1, PROTOCOL_2 = 2, UNDEF_PTC
};


vector<Model> p1_models = {X6_7, X6_40, X8_25, X6_8, X8_20, X8_60, X10_40, X10_100};
vector<Model> p2_models = {X4_24, X8_90, X12_150, X15_400};

// List of functions
int openSocket(const char* can_bus);
Protocol getProtocol();
void pingMotor(int id, Protocol protocol, int s);
void changeID(int currentID, int newID, Protocol protocol, int s);
void changeID_p1(int currentID, int newID, int s);
void changeID_p2(int currentID, int newID, int s);
void reenableCANFilter(int newID, int s);
timespec time_s();
double get_delta_us(struct timespec t2, struct timespec t1);

int main()
{
    cout << "This program allows to change the ID of a MyActuator motor" << endl << endl;
    cout << "=========== WARNING ===========" << endl;
    cout << "Make sure there is only ONE motor connected" << endl;
    cout << "Ready to continue? [y/n]" << endl;

    char c;
    while (c != 'y' && c != 'n') {
        cin >> c;

        if (c == 'y')
            break;
        else if (c == 'n') {
            cout << "Exiting" << endl;
            exit(1);
        }
    }
    cout << endl;

    const char* can_bus = "can0";
    int s = openSocket(can_bus);

    Protocol protocol = getProtocol();
    cout << "Protocol = " << protocol << endl;

    // Get current ID
    cout << endl << "What is the motor's current ID?" << endl;
    int currentID = -1;
    cin >> currentID;

    // Get new ID
    cout << endl << "What is the new ID you want to assign?" << endl;
    int newID = -1;
    cin >> newID;    

    // Summary + confirm
    cout << endl << "The parameters you entered are:" << endl;
    cout << "\tCurrent ID: " << currentID << endl;
    cout << "\tNew ID: " << newID << endl;
    cout << "Is this correct? [y/n]" << endl;

    while(1) {
        char userInput;
        cin >> userInput;

        if (userInput == 'y')
            break;
        else if (userInput == 'n'){
            cout << "Exiting" << endl;
            exit(1);
        }
    } 
    cout << endl;

    // Check if motor responding (depends on protocol)
    pingMotor(currentID, protocol, s);

    // Change id (depends on protocol) + reset
    changeID(currentID, newID, protocol, s);

    // check if motor with new ID responding (depends on protocol)
    pingMotor(newID, protocol, s);

    if (protocol == PROTOCOL_1)
        reenableCANFilter(newID, s);

    // Confirm (or deny) success
    cout << endl << "The motor's ID has successfully been changed to " << newID << endl;
}

int openSocket(const char* can_bus)
{
    int s = socket(PF_CAN, SOCK_RAW|SOCK_NONBLOCK, CAN_RAW);
    if (s < 0) {
        cout << "Error opening the socket! Exiting" << endl;
        exit(1);
    }
    else   
        cout << "Socket opened successfully" << endl;

    struct sockaddr_can addr;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_bus);
    ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));  // Init
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int result = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (result < 0) {
        cout << "Binding error! Exiting" << endl;
        exit(1);
    }
    else
        cout << "Binding ok " << endl;

    return s;
}

Protocol getProtocol()
{
   // Get info from user
    cout << "What is the model of your motor? " << endl;
    cout << "\t0: X6-7" << endl;
    cout << "\t1: X6-40" << endl;
    cout << "\t2: X8-25" << endl;
    cout << "\t3: X6-8" << endl;
    cout << "\t4: X8-20" << endl;
    cout << "\t5: X8-60" << endl;
    cout << "\t6: X10-40" << endl;
    cout << "\t7: X10-100" << endl;
    cout << "\t8: X4-24" << endl;
    cout << "\t9: X8-90" << endl;
    cout << "\t10: X12-150" << endl;
    cout << "\t11: X15-400" << endl;

    int userInput = -1;
    cin >> userInput;

    Model model = UNDEF_MODEL;

    switch (userInput)
    {
    case 0: model = X6_7; break; 
    case 1: model = X6_40; break;
    case 2: model = X8_25; break; 
    case 3: model = X6_8; break; 
    case 4: model = X8_20; break; 
    case 5: model = X8_60; break; 
    case 6: model = X10_40; break; 
    case 7: model = X10_100; break; 
    case 8: model = X4_24; break; 
    case 9: model = X8_90; break; 
    case 10: model = X12_150; break; 
    case 11: model = X15_400; break; 

    default:
        cout << "Error! Unrecognized model" << endl; 
        exit(1);
        break;
    }

    Protocol protocol = UNDEF_PTC;
    int cnt = std::count(p1_models.begin(), p1_models.end(), model);
    if (cnt > 0) 
        protocol = PROTOCOL_1;
    else {
        cnt = std::count(p2_models.begin(), p2_models.end(), model);

        if (cnt > 0)
            protocol = PROTOCOL_2;
        else {
            cout << "Error! Could not identify the protocol of the motor" << endl;
            exit(1);
        }
    }

    return protocol;
}


void pingMotor(int id, Protocol protocol, int s)
{
    if (protocol == PROTOCOL_1) {
        struct can_frame frame;
        frame.can_id = 0x140 + id;
        frame.len = 8;          // 8 bytes
        frame.data[0] = 0xB5;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;

        // Send frame
        int nbytes = write(s, &frame, sizeof(can_frame));
        if (nbytes < 0) {
            cout << "Problem with getting model" << endl;
            exit(1);
        }

        timespec start = time_s();
        while(1) {
    		// Check for timeout
            timespec end = time_s();
            double elapsed = get_delta_us(end, start);
            if (elapsed > RESPONSE_TIMEOUT) {
                cout << "Error! No response from motor. Check the ID and the model" << endl;
                exit(1);
            }

            can_frame frame;
            int nbytes = read(s, &frame, sizeof(can_frame));

            if (nbytes > 0) {
                if (frame.data[0] == 0xB5 && frame.can_id == 0x240+id) {
                    //cout << "Motor pinged successfully" << endl;
                    break;
                }
            }
        }
    }
    else {
        struct can_frame frame;
        frame.can_id = 0x7FF;
        frame.len = 4;
        frame.data[0] = 0xFF;
        frame.data[1] = 0xFF;
        frame.data[2] = 0x00;
        frame.data[3] = 0x82;

        // Send frame
        int nbytes = write(s, &frame, sizeof(can_frame));
        if (nbytes < 0) {
            cout << "Problem with getting model" << endl;
            exit(1);
        }

        nbytes = -1;
        frame.can_id = 0x00;
        for (int i=0; i<8; i++)
            frame.data[i] = 0x00;

        // Receive frame
        timespec start = time_s();
        while(1) {
    		// Check for timeout
            timespec end = time_s();
            double elapsed = get_delta_us(end, start);
            if (elapsed > RESPONSE_TIMEOUT) {
                cout << "Error! No response from motor. Check the ID and the model" << endl;
                exit(1);
            }

            nbytes = read(s, &frame, sizeof(can_frame));

            if (nbytes > 0) {
                int recID = frame.data[3]<<8 + frame.data[4];
                if (frame.data[0] == 0xFF && recID == id) {
                    //cout << "Motor pinged successfully" << endl;
                    break;
                }
            }
        }
    }
}

void changeID(int currentID, int newID, Protocol protocol, int s)
{
    if (protocol == PROTOCOL_1)
        changeID_p1(currentID, newID, s);
    else if (protocol == PROTOCOL_2)
        changeID_p2(currentID, newID, s);
}

void changeID_p1(int currentID, int newID, int s)
{
    if (newID < 1 || newID > 32) {
        cout << "Error! Protocol 1 only accepts IDs between 1 and 32" << endl;
        exit(1);
    }

    cout << "Setting ID..." << endl;

    // Disable can filter
    struct can_frame frame;
    frame.can_id = 0x140 + currentID;
    frame.len = 8;
    frame.data[0] = 0x20;
    frame.data[1] = 0x02; // CAN filter mode
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0;    // disable
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(s, &frame, sizeof(can_frame));
    if (nbytes < 0) {
        cout << "Error! Could not disable CAN filter" << endl;
        exit(1);
    }
 
    timespec start = time_s();
    while(1) {
        // Check for timeout
        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        if (elapsed > RESPONSE_TIMEOUT) {
            cout << "Error! No response from motor. Check the ID and the model" << endl;
            exit(1);
        }

        can_frame frame;
        int nbytes = read(s, &frame, sizeof(can_frame));

        if (nbytes > 0) {
            if (frame.data[0] == 0x20 && frame.can_id == 0x240+currentID) {
                //cout << "CAN filter disabled" << endl;
                break;
            }
        }
    }


    // Set the new ID
    frame.can_id = 0x300;
    frame.len = 8;
    frame.data[0] = 0x79;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00; // Write CAN ID
    frame.data[3] = 0x00;
    frame.data[4] = 0x00; 
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = newID;

    // Send frame
    nbytes = -1;
    nbytes = write(s, &frame, sizeof(can_frame));
    if (nbytes < 0) {
        cout << "Error! Could not send ID command" << endl;
        exit(1);
    }
 
    start = time_s();
    while(1) {
        // Check for timeout
        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        if (elapsed > RESPONSE_TIMEOUT) {
            cout << "Error! No response from motor. Check the ID and the model" << endl;
            exit(1);
        }
        can_frame frame;
        int nbytes = read(s, &frame, sizeof(can_frame));

        if (nbytes > 0) {
            if (frame.data[0] == 0x79 && frame.can_id == 0x300) {
                //cout << "ID changed" << endl;
                break;
            }
        }
    }

    // Reset motor
    frame.can_id = 0x140+newID;
    frame.len = 8;
    frame.data[0] = 0x76;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    nbytes = write(s, &frame, sizeof(can_frame));
    if (nbytes < 0) {
        cout << "Error! Could not send reset command" << endl;
        exit(1);
    }

    usleep(3*1000000);
}

void changeID_p2(int currentID, int newID, int s)
{
    // Reset to ID 1 first, as recommended in the documentation
    struct can_frame frame;
    frame.can_id = 0x7FF;
    frame.len = 6;
    frame.data[0] = (uint8_t) (currentID>>8);
    frame.data[1] = (uint8_t) currentID;
    frame.data[2] = 0x00;
    frame.data[3] = 0x04;
    frame.data[4] = (uint8_t) (1>>8);
    frame.data[5] = (uint8_t) 1;

    // Send frame
    int nbytes = -1;
    nbytes = write(s, &frame, sizeof(can_frame));
    if (nbytes < 0) {
        cout << "Error! Could not change ID to 1, in protocol 2" << endl;
        exit(1);
    }
 
    timespec start = time_s();
    while(1) {
        // Check for timeout
        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        if (elapsed > RESPONSE_TIMEOUT) {
            cout << "Error! Could not change ID to 1, in protocol 2" << endl;
            exit(1);
        }

        can_frame frame;
        int nbytes = read(s, &frame, sizeof(can_frame));

        if (nbytes > 0) {
            int recID = (int)frame.data[0]<<8 + (int)frame.data[1];
            if (frame.data[3] == 0x04 && recID == 1) {
                cout << "ID reset to 1" << endl;
                break;
            }
        }
    }

    // Now set the ID to the request new ID
    frame.can_id = 0x7FF;
    frame.len = 6;
    frame.data[0] = (uint8_t) (1>>8);
    frame.data[1] = (uint8_t) 1;
    frame.data[2] = 0x00;
    frame.data[3] = 0x04;
    frame.data[4] = (uint8_t) (newID>>8);
    frame.data[5] = (uint8_t) newID;

    // Send frame
    nbytes = write(s, &frame, sizeof(can_frame));
    if (nbytes < 0) {
        cout << "Error! Could not change ID to " << newID << ", in protocol 2" << endl;
        exit(1);
    }
 
    start = time_s();
    while(1) {
        // Check for timeout
        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        if (elapsed > RESPONSE_TIMEOUT) {
            cout << "Error! Could not change ID to " << newID << ", in protocol 2" << endl;
            exit(1);
        }

        can_frame frame;
        int nbytes = read(s, &frame, sizeof(can_frame));

        if (nbytes > 0) {
            int recID = (int)frame.data[0]<<8 + (int)frame.data[1];
            if (frame.data[3] == 0x04 && recID == newID) {
                cout << "ID set to " << newID << endl;
                break;
            }
        }
    }
}


void reenableCANFilter(int id, int s)
{
    // Enable can filter
    struct can_frame frame;
    frame.can_id = 0x140 + id;
    frame.len = 8;
    frame.data[0] = 0x20;
    frame.data[1] = 0x02; // CAN filter mode
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 1;    // enable
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    // Send frame
    int nbytes = -1;
    nbytes = write(s, &frame, sizeof(can_frame));
    if (nbytes < 0) {
        cout << "Error! Could not enable CAN filter" << endl;
        exit(1);
    }
 
    timespec start = time_s();
    while(1) {
        // Check for timeout
        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        if (elapsed > RESPONSE_TIMEOUT) {
            cout << "Error! No response from motor. Check the ID and the model" << endl;
            exit(1);
        }

        can_frame frame;
        int nbytes = read(s, &frame, sizeof(can_frame));

        if (nbytes > 0) {
            if (frame.data[0] == 0x20 && frame.can_id == 0x240+id) {
                //cout << "CAN filter enabled for motor " << id << endl;
                break;
            }
        }
    }
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