/**
 **********************************************************************************************************************
 * @file       WjwwoodSerial.h
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

#ifndef HaplyWjwwoodSerial
#define HaplyWjwwoodSerial

#include "Haply/Serial.h"

#include "serial/serial.h"

namespace Haply
{

class WjwwoodSerial: public Haply::Serial
{

public:
    WjwwoodSerial();

    virtual void clear();
    virtual bool available();
    virtual bool open();
    virtual bool close();
    virtual void buffer(int length);
    virtual void readBytes(std::vector<byte>& inData);
    virtual void write(std::vector<byte> outData);
private:
    serial::Serial* my_serial;
    int bufferSize;
};

}

#endif // HaplyWjwwoodSerial
