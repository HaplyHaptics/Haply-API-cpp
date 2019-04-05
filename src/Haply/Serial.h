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

class Serial
{

public:
    Serial() {}

    virtual void clear() = 0;
    virtual bool available() = 0;
    virtual bool open() = 0;
    virtual bool close() = 0;
    virtual void buffer(int length) = 0;
    virtual void readBytes(std::vector<byte> &inData) = 0;
    virtual void write(std::vector<byte> outData) = 0;
};

} // namespace Haply

#endif // __HaplySerial__
