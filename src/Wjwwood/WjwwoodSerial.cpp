/**
**********************************************************************************************************************
* @file       WjwwoodSerial.cpp
* @author     Christian Frisson
* @version    V0.1.0
* @date       4-January-2018
* @brief      Serial port class implementation
**********************************************************************************************************************
* @attention
*
*
**********************************************************************************************************************
*/

#include "Wjwwood/WjwwoodSerial.h"

#include <string>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

using namespace Haply;
using namespace std;

void my_sleep(unsigned long milliseconds)
{
#ifdef _WIN32
    Sleep(milliseconds);
#else
    usleep(milliseconds*1000);
#endif
}

void enumerate_ports()
{
    vector<serial::PortInfo> devices_found = serial::list_ports();

    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while( iter != devices_found.end() ) {
        serial::PortInfo device = *iter++;

        printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
                device.hardware_id.c_str() );
    }
}

WjwwoodSerial::WjwwoodSerial()
    : Serial()
{
    my_serial = 0;
    bufferSize = 0;
}

void WjwwoodSerial::clear()
{
    if(!my_serial) {
        std::cerr << "Serial port not created" << std::endl;
        return;
    }
    my_serial->flush();
}

bool WjwwoodSerial::available()
{
    if(!my_serial) {
        std::cerr << "Serial port not created" << std::endl;
        return false;
    }
    bool _available = false;

    try {
        _available = my_serial->available();
    } catch(serial::IOException e) {
        std::cerr << "Error when checking if data is available: " << e.what() << std::endl;
    }
    return _available;
}

bool WjwwoodSerial::open()
{
    if(!my_serial) {
        // Forcing default settings
        std::string portName;
        int baud;
        baud = 9600;
#ifdef __APPLE__
        std::string portNameMatch = "/dev/cu.usbmodem";
#elif defined(__linux__)
        std::string portNameMatch = "/dev/ttyACM";
#else
        std::string portNameMatch = "COM";
#endif
        vector<serial::PortInfo> devices_found = serial::list_ports();
        vector<serial::PortInfo>::iterator iter = devices_found.begin();
        while( iter != devices_found.end() ) {
            serial::PortInfo device = *iter++;
            int cmp = device.port.find(portNameMatch) != std::string::npos;
            //printf( "(%s, %s, %s) =  %d\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str(),cmp );
            if(cmp > 0) {
                portName = device.port;
            }
        }
        if(portName.empty()) {
            std::cerr << "No matching serial port device, aborting." << std::endl;
            return false;
        }
        //std::cout << "Opening " << portName << std::endl;

        // port, baudrate, timeout in milliseconds
        try {
            my_serial = new serial::Serial(portName, baud, serial::Timeout::simpleTimeout(1000));
        } catch(serial::IOException e) {
            std::cerr << "Could not open serial port: " << e.what() << std::endl;
            my_serial = 0;
            return false;
        }

        //cout << "Is the serial port open?";
        // if(my_serial && my_serial->isOpen())
        //   cout << " Yes." << endl;
        // else
        //   cout << " No." << endl;

    }

    if(!my_serial) {
        std::cerr << "Serial port not created" << std::endl;
        return false;
    }
    return my_serial->isOpen();
}

bool WjwwoodSerial::close()
{
    if(my_serial) {
        delete my_serial;
        my_serial = 0;
        return true;
    }
    return false;
}

void WjwwoodSerial::buffer(int length)
{
    if(!my_serial) {
        std::cerr << "Serial port not created" << std::endl;
        return;
    }
    //my_serial->setBytesize((serial::bytesize_t)length);
    this->bufferSize = length;
}

void WjwwoodSerial::readBytes(std::vector<byte>& inData)
{
    if(!my_serial) {
        std::cerr << "Serial port not created" << std::endl;
        return;
    }
    try {
        my_serial->read(inData, this->bufferSize);
    } catch(serial::SerialException e) {
        std::cerr << "Error when checking if data is available: " << e.what() << std::endl;
    }
}

void WjwwoodSerial::write(std::vector<byte> outData)
{
    if(!my_serial) {
        std::cerr << "Serial port not created" << std::endl;
        return;
    }
    try {
        size_t w = my_serial->write(outData);
        //std::cerr << "Wrote " << w << " bytes to device" << std::endl;
    } catch(serial::SerialException e) {
        std::cerr << "Error when checking if data is available: " << e.what() << std::endl;
    }
}
