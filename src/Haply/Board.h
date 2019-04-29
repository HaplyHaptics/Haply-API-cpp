/**
 **********************************************************************************************************************
 * @file       Board.h
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V2.1.0
 * @date       17-April-2019
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
    byte device_id;
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
       * @param     communication_type type of communication taking place
       * @param     device_id ID of device transmitting the information
       * @param     b_data byte information to be transmitted  * @param     f_data float information to be transmitted
       */
public:
    void transmit(byte communication_type, byte device_id, std::vector<byte> b_data, std::vector<float> f_data);

    /**
       * Receives data from the serial port and formats data to return a float data array
       *
       * @param     type type of communication taking place
       * @param     device_id ID of the device receiving the information
       * @param     expected number for floating point numbers that are expected
       * @return    formatted float data array from the received data
       */
public:
    std::vector<float> receive(byte communication_type, byte device_id, int expected);

    /**
       * @return   a bool indicating if data is available from the serial port
       */
public:
    bool data_available();

    /**
      * Sends a reset command to perform a software reset of the Haply board
      *
      */
private:
    void reset_board();

    /**
       * Set serial buffer length for receiving incoming data
       *
       * @param   length number of bytes expected in read buffer
       */
private:
    void set_buffer(int length);

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

}; // class Board

} // namespace Haply

#endif // __HaplyBoard__
