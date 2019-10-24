/**
 **********************************************************************************************************************
 * @file       Board.cpp
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

#include <iostream>

#include "Haply/Board.h"

#include <cmath>

using namespace Haply;
using namespace std;

/**
 * Returns the raw 32-bit integer representation of an IEEE 754 floating-point scalar or vector
 * From Nvidia Cg 3.1 Toolkit Documentation / Cg / Standard Library
 * http://developer.download.nvidia.com/cg/floatToRawIntBits.html
 *
 */
int floatToRawIntBits(float x)
{
    union {
        float f; // assuming 32-bit IEEE 754 single-precision
        int i;   // assuming 32-bit 2's complement int
    } u;
    u.f = x;
    return u.i;
}

/**
 * Returns the IEEE 754 floating-point scalar representation of a 32-bit integer
 * From Nvidia Cg 3.1 Toolkit Documentation / Cg / Standard Library
 * http://developer.download.nvidia.com/cg/intBitsToFloat.html
 *
 */
float intBitsToFloat(int x)
{
    union {
        float f; // assuming 32-bit IEEE 754 single-precision
        int i;   // assuming 32-bit 2's complement int
    } u;
    u.i = x;
    return u.f;
}

/**
 * Constructs a Board linking to the specified serial communication object
 *
 * @param    serial communication object
 */
Board::Board(Serial *serial_com)
    : number_of_parameters(0), port(0)
{
    std::fill(actuator_positions, actuator_positions + 4, 0);
    port = serial_com;
    port->clear();
    this->reset_board();
}

/**
 * Formats and transmits the float data array over the serial port
 *
 * @param     communication_type type of communication taking place
 * @param     device_id ID of device transmitting the information
 * @param     b_data byte information to be transmitted
 * @param     f_data float information to be transmitted
 */
void Board::transmit(byte communication_type, byte device_id, std::vector<byte> b_data, std::vector<float> f_data)
{
    std::vector<byte> out_data,segments;
    out_data.resize(2 + b_data.size() + 4 * f_data.size());

    out_data[0] = communication_type;
    out_data[1] = device_id;
    this->device_id = device_id;

    std::copy(b_data.begin(),b_data.end(),out_data.begin()+2);

    int j = 2 + b_data.size();
    for (int i = 0; i < f_data.size(); i++) {
        std::vector<byte> segments;
        segments = FloatToBytes(f_data[i]);
        std::copy( segments.begin(), segments.end(), out_data.begin() + j);
        j = j + 4;
    }

    this->port->write(out_data);
}

/**
 * Receives data from the serial port and formats data to return a float data array
 * @param     type type of communication taking place
 * @param     device_id ID of the device receiving the information
 * @param     expected number for floating point numbers that are expected
 * @return    formatted float data array from the received data
 */
std::vector<float> Board::receive(byte communication_type, byte device_id, int expected)
{

    set_buffer(1 + 4*expected);

    std::vector<byte> in_data;
    in_data.reserve(1 + 4*expected);

    std::vector<float> data;
    data.resize(expected);

    this->port->readBytes(in_data);

    if (in_data[0] != device_id) {
        std::cerr << "Error, another device expects this data!" << std::endl;
    }

    int j = 1;

    for (int i = 0; i < expected; i++) {
        std::vector<byte> segments;
        segments.resize(4);
        std::copy( in_data.begin() + j, in_data.begin() + j + 4, segments.begin());
        data[i] = BytesToFloat(segments);
        j = j + 4;
    }

    return data;
}

/**
 * @return   a bool indicating if data is available from the serial port
 */
bool Board::data_available()
{

    bool available = false;

    if (port->available() > 0) {
        available = true;
    }

    return available;
}

/**
  * Sends a reset command to perform a software reset of the Haply board
  *
  */
void Board::reset_board()
{
    byte communication_type = 0;
    byte device_id = 0;
    std::vector<byte> b_data(0);
    std::vector<float> f_data(0);

    transmit(communication_type, device_id, b_data, f_data);
}

/**
 * Set serial buffer length for receiving incoming data
 *
 * @param   length number of bytes expected in read buffer
 */
void Board::set_buffer(int length)
{
    this->port->buffer(length);
}

/**
 * Translates a float point number to its raw binary format and stores it across four bytes
 *
 * @param    val floating point number
 * @return   array of 4 bytes containing raw binary of floating point number
 */
std::vector<byte> Board::FloatToBytes(float val)
{

    std::vector<byte> segments(4, 0);

    int temp = floatToRawIntBits(val);

    segments[3] = (byte)((temp >> 24) & 0xff);
    segments[2] = (byte)((temp >> 16) & 0xff);
    segments[1] = (byte)((temp >> 8) & 0xff);
    segments[0] = (byte)((temp)&0xff);

    return segments;
}

/**
 * Translates a binary of a float point to actual float point
 *
 * @param    segment array containing raw binary of floating point
 * @return   translated floating point number
 */
float Board::BytesToFloat(std::vector<byte> segment)
{

    int temp = 0;

    temp = (temp | (segment[3] & 0xff)) << 8;
    temp = (temp | (segment[2] & 0xff)) << 8;
    temp = (temp | (segment[1] & 0xff)) << 8;
    temp = (temp | (segment[0] & 0xff));

    float val = intBitsToFloat(temp);

    return val;
}
