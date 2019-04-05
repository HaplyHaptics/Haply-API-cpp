/**
 **********************************************************************************************************************
 * @file       Board.h
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V0.1.0
 * @date       01-March-2017
 * @brief      Board class definition
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#ifndef __HaplyBoard__
#define __HaplyBoard__

#include "Haply/Serial.h"

#include <string>
#include <vector>

namespace Haply
{

class Board
{

    Serial *port;

private:
    byte deviceID;
    int number_of_parameters;
    byte actuator_positions[4];

    /**
       * Constructs a Board linking to the specified serial port at the given serial data speed (baud rate)
       *
       * @param    app the parent Applet this->class runs inside (this->is your Processing sketch)
       */
public:
    Board(Serial *serial_com);

    /**
       * Formats and transmits the float data array over the serial port
       *
       * @param     type type of communication taking place
       * @param     deviceID ID of device transmitting the information
       * @param     positions the motor positions the data is meant for
       * @param     data main data payload to be transmitted
       */
public:
    void transmit(byte type, byte deviceID, std::vector<byte> positions, std::vector<float> data);

    /**
       * Receives data from the serial port and formats said data to return a float data array
       *
       * @param     type type of communication taking place
       * @param     deviceID ID of the device receiving the information
       * @param     positions the motor positions the data is meant for
       * @return    formatted float data array from the received data
       */
public:
    std::vector<float> receive(byte type, byte deviceID, std::vector<byte> positions);

    /**
       * @return   a bool indicating if data is available from the serial port
       */
public:
    bool data_available();

    /**
       * @return   a bool indicating if the board is available from the serial port
       */
public:
    bool board_available();

    /**
       * Set serial buffer length for receiving incoming data
       *
       * @param   length number of bytes expected in read buffer
       */
private:
    void set_buffer(int length);

    /**
       * Determines how much data should incoming and sets buffer lengths accordingly
       *
       * @param    type type of communication taking place
       * @param    positions the motor positions the data is meant for
       * @return   number of active motors
       */
private:
    int set_buffer_length(byte type, std::vector<byte> positions);

    /**
       * Determines if actuator ports are in use and prints warnings accordingly
       *
       * @param    positions the motor positions being set
       */
private:
    void port_check(std::vector<byte> positions);

    /**
       * Formats header control byte for transmission over serial
       *
       * @param    type type of communication taking place
       * @param    positions the motor positions the data is meant for
       * @return   formatted header control byte
       */
private:
    byte format_header(byte type, std::vector<byte> positions);

    /**
       * Translates a float point number to its raw binary format and stores it across four bytes
       *
       * @param    val floating point number
       * @return   array of 4 bytes containing raw binary of floating point number
       */
private:
    std::vector<byte> FloatToBytes(float val);

    /**
       * Translates a binary of a float point to actual float point
       *
       * @param    segment array containing raw binary of floating point
       * @return   translated floating point number
       */
private:
    float BytesToFloat(std::vector<byte> segment);
};

} // namespace Haply

#endif // __HaplyBoard__
