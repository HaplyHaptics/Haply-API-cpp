/**
**********************************************************************************************************************
* @file       WjwwoodSerial.cpp
* @author     Christian Frisson
* @version    V0.1.0
* @date       4-January-2018
* @brief      Serial port class implementation with Wjwwood's serial library
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

/**
 * Constructs a serial communication object implemented with the Wjwwood's serial library
 */
WjwwoodSerial::WjwwoodSerial()
    : Serial()
{
    m_serial = 0;
    m_buffer_size = 0;
}

/**
 * Empties the buffer, removes all the data stored there
 */
void WjwwoodSerial::clear()
{
    if(!m_serial) {
        std::cerr << "Serial port not created" << std::endl;
        return;
    }
    m_serial->flush();
}

/**
 * Tests if the serial port is available
 *
 * @return   a boolean indicating if the serial port is available
 */
bool WjwwoodSerial::available()
{
    if(!m_serial) {
        std::cerr << "Serial port not created" << std::endl;
        return false;
    }
    bool _available = false;

    try {
        _available = m_serial->available();
    } catch(serial::IOException e) {
        std::cerr << "Error when checking if data is available: " << e.what() << std::endl;
    }
    return _available;
}

/**
 * Opens the serial port with the specified serial port name and at the given serial data speed (baud rate)
 *
 * @param    port_name serial port name that the hardware board is connected to
 * @param    baud_rate the baud rate of serial data transfer
 */
bool WjwwoodSerial::open(std::string port_name, int baud_rate)
{
    if(!m_serial) {
        std::string _port_name,port_name_match;
        if(port_name.empty()) {
            // Forcing default settings
#ifdef __APPLE__
            port_name_match = "/dev/cu.usbmodem";
#elif defined(__linux__)
            port_name_match = "/dev/ttyACM";
#else
            port_name_match = "COM";
#endif
        } else {
            // Trying expected port name
            port_name_match = port_name;
        }
        vector<serial::PortInfo> devices_found = serial::list_ports();
        vector<serial::PortInfo>::iterator iter = devices_found.begin();
        while( iter != devices_found.end() ) {
            serial::PortInfo device = *iter++;
            int cmp = device.port.find(port_name_match) != std::string::npos;
            //printf( "(%s, %s, %s) =  %d\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str(),cmp );
            if(cmp > 0) {
                _port_name = device.port;
            }
        }
        if(_port_name.empty()) {
            std::cerr << "No matching serial port device '" << port_name_match << "', aborting." << std::endl;
            return false;
        }
        //std::cout << "Opening " << _port_name << std::endl;

        // port, baudrate, timeout in milliseconds
        try {
            m_serial = new serial::Serial(_port_name, baud_rate, serial::Timeout::simpleTimeout(1000));
        } catch(serial::IOException e) {
            std::cerr << "Could not open serial port: " << e.what() << std::endl;
            m_serial = 0;
            return false;
        }

        //cout << "Is the serial port open?";
        // if(m_serial && m_serial->isOpen())
        //   cout << " Yes." << endl;
        // else
        //   cout << " No." << endl;

    }

    if(!m_serial) {
        std::cerr << "Serial port not created" << std::endl;
        return false;
    }
    return m_serial->isOpen();
}

/**
 * Closes the serial port
 *
 * @return   a boolean indicating if the serial port has been closed
 */
bool WjwwoodSerial::close()
{
    if(m_serial) {
        delete m_serial;
        m_serial = 0;
        return true;
    }
    return false;
}

/**
 * Sets the number of bytes to buffer
 *
 * @param   length buffer size
 */
void WjwwoodSerial::buffer(int length)
{
    if(!m_serial) {
        std::cerr << "Serial port not created" << std::endl;
        return;
    }
    //m_serial->setBytesize((serial::bytesize_t)length);
    this->m_buffer_size = length;
}

/**
 * Reads a group of bytes in a buffer
 *
 * @param   in_data buffer to store the bytes
 */
void WjwwoodSerial::readBytes(std::vector<byte>& in_data)
{
    if(!m_serial) {
        std::cerr << "Serial port not created" << std::endl;
        return;
    }
    try {
        //std::cerr << "Reading... " << std::endl;
        size_t r = m_serial->read(in_data, this->m_buffer_size);
        //std::cerr << "Read " << r << " bytes from device, expected buffer size of " << this->m_buffer_size << std::endl;
    } catch(serial::SerialException e) {
        std::cerr << "Error when checking if data is available: " << e.what() << std::endl;
    }
}

/**
 * Writes a group of bytes to the serial port
 *
 * @param   out_data buffer of bytes to write
 */
void WjwwoodSerial::write(std::vector<byte> out_data)
{
    if(!m_serial) {
        std::cerr << "Serial port not created" << std::endl;
        return;
    }
    try {
        // std::cerr << "Writing... " << std::endl;
        size_t w = m_serial->write(out_data);
        // std::cerr << "Wrote " << w << " bytes to device" << std::endl;
    } catch(serial::SerialException e) {
        std::cerr << "Error when checking if data is available: " << e.what() << std::endl;
    }
}
