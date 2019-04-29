/**
**********************************************************************************************************************
* @file       example main.cpp
* @author     Christian Frisson
* @version    V0.1.0
* @date       23-April-2019
* @brief      Simple example to communicate with the Haply 2-DOF Pantograph
**********************************************************************************************************************
* @attention
*
*
**********************************************************************************************************************
*/

#include "Haply/Board.h"
#include "Haply/Device.h"
#include "Haply/Pantograph.h"
#include "Haply/Types.h"
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
    /* Device block definitions */
    Serial *serial_com = 0;
    Board *haply_board = 0;
    Device *widget_one = 0;
    Mechanisms *pantograph = 0;

    byte widget_one_id = 5;
    int cw = 0;
    int ccw = 1;

    /* Device I/O values */
    std::vector<float> torques(2,0.0);
    std::vector<float> angles;
    std::vector<float> positions;

    /* device setup */

    /* Serial communication */
    serial_com = new WjwwoodSerial();
    int baud_rate = 0;

    /**
     * The serial communication object needs to open the USB serial port the Haply board is connected.
     * Two options:
     * 1) specify the desired port name,
     * 2) let the object match the port name from the following os-dependent port name stems:
     *    linux: stem "/dev/ttyACM", default "/dev/ttyACM0"
     *    mac: stem "/dev/cu.usbmodem", default "/dev/cu.usbmodem14101"
     *    windows: stem "COM", default "COM10"
     */
    std::string port_name;
#ifdef __APPLE__
    port_name = "/dev/cu.usbmodem14011";
#elif defined(__linux__)
    port_name = "/dev/ttyACM0";
#else
    port_name = "COM10";
#endif
    if (!serial_com || !serial_com->open(port_name,baud_rate)) {
        std::cerr << "Couldn't create serial communication with Haply board" << std::endl;
        return 0;
    }

    /* Board */
    haply_board = new Board(serial_com);
    if (!haply_board) {
        std::cerr << "Couldn't create Haply board" << std::endl;
        return 0;
    }

    /* Device */
    widget_one = new Device(widget_one_id, haply_board);
    if (!widget_one) {
        std::cerr << "Couldn't create Haply device" << std::endl;
        return 0;
    }

    /* Mechanism */
    pantograph = new Pantograph();
    widget_one->set_mechanism(pantograph);
    widget_one->add_actuator(1, cw, 1);
    widget_one->add_actuator(2, cw, 2);
    widget_one->add_encoder(1, cw, 180, 13824, 1);
    widget_one->add_encoder(2, cw, 0, 13824, 2);
    widget_one->device_set_parameters();

    /* Simulation section */
    long i = 0;
    while (1) {
        if (haply_board->data_available()) {
            widget_one->device_read_data();
            angles = widget_one->get_device_angles();
            positions = widget_one->get_device_position(angles);

            std::cout << "Data available @ frame " << i << ": ";
            std::cout << "angles(" << angles[0] << "," << angles[1] << ") ";
            std::cout << "positions(" << positions[0] << "," << positions[1] << ") ";
            std::cout << std::endl;
        } else {
            std::cout << "Data not available @ frame " << i << std::endl;
        }
        widget_one->set_device_torques(torques);
        widget_one->device_write_torques();

        i++;
        _sleep(1); // adjust this sleep value to pace messages
    }
}
