/**
 **********************************************************************************************************************
 * @file       Board.cpp
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
}

/**
 * Formats and transmits the float data array over the serial port
 *
 * @param     type type of communication taking place
 * @param     deviceID ID of device transmitting the information
 * @param     positions the motor positions the data is meant for
 * @param     data main data payload to be transmitted
 */
void Board::transmit(byte type, byte deviceID, std::vector<byte> positions, std::vector<float> data)
{

    std::vector<byte> outData;
    outData.reserve(2 + 4 * sizeof(data));

    outData.insert(outData.begin(), format_header(type, positions));
    outData.insert(outData.begin() + 1, deviceID);
    this->deviceID = deviceID;

    int j = 2;
    for (int i = 0; i < data.size(); i++) {
        std::vector<byte> segments;
        segments.reserve(4);
        segments = FloatToBytes(data[i]);
        //System.arraycopy(segments, 0, outData, j, 4);
        outData.insert(outData.begin() + j, segments.begin(), segments.end());
        j = j + 4;
    }

    this->port->write(outData);
}

/**
 * Receives data from the serial port and formats said data to return a float data array
 *
 * @param     type type of communication taking place
 * @param     deviceID ID of the device receiving the information
 * @param     positions the motor positions the data is meant for
 * @return    formatted float data array from the received data
 */
std::vector<float> Board::receive(byte type, byte deviceID, std::vector<byte> positions)
{

    int size = set_buffer_length(type, positions);

    std::vector<byte> inData;
    inData.reserve(1 + 4 * size);

    std::vector<float> data(size);

    this->port->readBytes(inData);

    if (inData[0] != deviceID) {
        std::cerr << "Error, another device expects this data!" << std::endl;
    }

    int j = 1;

    for (int i = 0; i < size; i++) {
        std::vector<byte> segments;
        segments.reserve(4);
        //System.arraycopy(inData, j, segments, 0, 4);
        segments.insert(segments.begin(), inData.begin() + j, inData.begin() + j + 4);
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
 * @return   a bool indicating if the board is available from the serial port
 */
bool Board::board_available()
{

    return port ? port->open() : false;
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
 * Determines how much data should incoming and sets buffer lengths accordingly
 *
 * @param    type type of communication taking place
 * @param    positions the motor positions the data is meant for
 * @return   number of active motors
 */
int Board::set_buffer_length(byte type, std::vector<byte> positions)
{

    int m_active = 0;

    for (int i = 0; i < 4; i++) {
        if (positions[i] > 0) {
            m_active++;
        }
    }

    switch (type) {
    case 0: // setup command
        port_check(positions);
        set_buffer(5);
        m_active = 1;
        break;
    case 1: // read encoder data
        set_buffer(1 + 4 * m_active);
        break;
    }

    return m_active;
}

/**
 * Determines if actuator ports are in use and prints warnings accordingly
 *
 * @param    positions the motor positions being set
 */
void Board::port_check(std::vector<byte> positions)
{

    for (int i = 0; i < 4; i++) {
        if (actuator_positions[i] > 0 && positions[i] > 0) {
            std::cerr << "Warning, hardware actuator " << i << " was in use and will be overridden" << std::endl;
        }

        actuator_positions[i] = positions[i];
    }
}

/**
 * Formats header control byte for transmission over serial
 *
 * @param    type type of communication taking place
 * @param    positions the motor positions the data is meant for
 * @return   formatted header control byte
 */
byte Board::format_header(byte type, std::vector<byte> positions)
{

    int header = 0;

    for (int i = 0; i < positions.size(); i++) {

        header = header >> 1;

        if (positions[i] > 0) {
            header = header | 0x0008;
        }
    }

    header = header | (type << 4);

    return (byte)header;
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
