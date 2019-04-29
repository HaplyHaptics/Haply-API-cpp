/**
 **********************************************************************************************************************
 * @file       WjwwoodSerial.h
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

#ifndef HaplyWjwwoodSerial
#define HaplyWjwwoodSerial

#include "Haply/Serial.h"

#include "serial/serial.h"

namespace Haply
{

class WjwwoodSerial: public Haply::Serial
{

public:
    /**
     * Constructs a serial communication object implemented with the Wjwwood's serial library
     */
    WjwwoodSerial();

    /**
     * Empties the buffer, removes all the data stored there
     */

    virtual void clear();

    /**
     * Tests if the serial port is available
     *
     * @return   a boolean indicating if the serial port is available
     */
    virtual bool available();

    /**
     * Opens the serial port with the specified serial port name and at the given serial data speed (baud rate)
     *
     * @param    port_name serial port name that the hardware board is connected to
     * @param    baud_rate the baud rate of serial data transfer
     */
    virtual bool open(std::string port_name="", int baud_rate=0);

    /**
    * Closes the serial port
    *
    * @return   a boolean indicating if the serial port has been closed
    */
    virtual bool close();

    /**
    * Sets the number of bytes to buffer
    *
    * @param   length buffer size
    */
    virtual void buffer(int length);

    /**
    * Reads a group of bytes in a buffer
    *
    * @param   in_data buffer to store the bytes
    */
    virtual void readBytes(std::vector<byte>& in_data);

    /**
    * Writes a group of bytes to the serial port
    *
    * @param   out_data buffer of bytes to write
    */
    virtual void write(std::vector<byte> out_data);

private:
    serial::Serial* m_serial;
    int m_buffer_size;
};

}

#endif // HaplyWjwwoodSerial
