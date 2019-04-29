/**
 **********************************************************************************************************************
 * @file       Serial.h
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V0.1.0
 * @date       14-December-2017
 * @brief      Serial port class definition
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#ifndef __HaplySerial__
#define __HaplySerial__

#include <iostream>
#include "Haply/Types.h"

#include <vector>
#include <string>

namespace Haply
{

    /**
     * Haply Serial abstract class
     * 
     * Follows partially the Processing Serial library API: 
     * https://processing.org/reference/libraries/serial/
     */
class Serial
{

public:
    /**
     * Constructs a serial communication object
     */
    Serial() {}

    /**
     * Empties the buffer, removes all the data stored there
     */

    virtual void clear() = 0;

    /**
     * Tests if the serial port is available
     *
     * @return   a boolean indicating if the serial port is available
     */
    virtual bool available() = 0;

    /**
     * Opens the serial port with the specified serial port name and at the given serial data speed (baud rate)
     *
     * @param    port_name serial port name that the hardware board is connected to
     * @param    baud_rate the baud rate of serial data transfer
     */
    virtual bool open(std::string port_name="", int baud_rate=0) = 0;

    /**
    * Closes the serial port
    *
    * @return   a boolean indicating if the serial port has been closed
    */
    virtual bool close() = 0;

    /**
    * Sets the number of bytes to buffer
    *
    * @param   length buffer size
    */
    virtual void buffer(int length) = 0;

    /**
    * Reads a group of bytes in a buffer 
    *
    * @param   in_data buffer to store the bytes
    */
    virtual void readBytes(std::vector<byte>& in_data) = 0;

    /**
    * Writes a group of bytes to the serial port
    *
    * @param   out_data buffer of bytes to write
    */
    virtual void write(std::vector<byte> out_data) = 0;
};

} // namespace Haply

#endif // __HaplySerial__
