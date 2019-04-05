/**
**********************************************************************************************************************
* @file       example main.cpp
* @author     Christian Frisson
* @version    V0.1.0
* @date       4-January-2018
* @brief      Simple example to communicate with the Haply 2-DOF Pantograph
**********************************************************************************************************************
* @attention
*
*
**********************************************************************************************************************
*/

#include "Haply/Board.h"
#include "Haply/Device.h"
#include "Wjwwood/WjwwoodSerial.h"

#include <iostream>

// OS-specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

void _sleep(unsigned long milliseconds)
{
#ifdef _WIN32
    Sleep(milliseconds);
#else
    usleep(milliseconds * 1000);
#endif
}

using namespace Haply;
using namespace std;

int main(int argc, char *argv[])
{

    byte deviceID = 5;
    DeviceType degreesOfFreedom;
    std::vector<float> angles;
    std::vector<float> pos_ee;
    std::vector<float> f_ee(3, 0);
    std::vector<float> torques(3, 0);

    /* SERIAL com */
    Serial *serial_com = 0;
    serial_com = new WjwwoodSerial();
    if (!serial_com || !serial_com->open()) {
        std::cerr << "Couldn't create serial communication with Haply board" << std::endl;
        return 0;
    }

    /* BOARD */
    Board *haply_board = new Board(serial_com);
    if (!haply_board || !haply_board->board_available()) {
        std::cerr << "Couldn't create Haply board" << std::endl;
        return 0;
    }

    /* DEVICE */
    Device *haply_2DoF = new Device(DeviceType::HaplyTwoDOF, deviceID, haply_board);
    if (!haply_2DoF) {
        std::cerr << "Couldn't create Haply device" << std::endl;
        return 0;
    }

    while (1) {
        if (haply_board->data_available()) {
            angles = haply_2DoF->get_device_angles();
            pos_ee = haply_2DoF->get_device_position(angles);
            std::cout << "Data available: ";
            std::cout << "angles(" << angles[0] << "," << angles[1] << ") ";
            std::cout << "pos_ee(" << pos_ee[0] << "," << pos_ee[1] << ") ";
            std::cout << std::endl;
        } else {
            std::cout << "Data not available" << std::endl;
        }
        haply_2DoF->set_device_torques(f_ee);
        haply_2DoF->device_write_torques();

        _sleep(1); // adjust this sleep value to pace messages
    }
}
