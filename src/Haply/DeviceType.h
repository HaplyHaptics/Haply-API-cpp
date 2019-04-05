/**
 **********************************************************************************************************************
 * @file       DeviceType.h
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V0.1.0
 * @date       01-March-2017
 * @brief      DeviceType enum definition
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#ifndef __HaplyDeviceType__
#define __HaplyDeviceType__

namespace Haply
{

/**
 * Identifies the device type and mechanism
 */
enum DeviceType {
    HaplyOneDOF = 0,
    HaplyTwoDOF = 1,
    HaplyThreeDOF = 2,
    HaplyFourDOF = 3,
    HapticPaddle = 4
};
}

#endif // __HaplyDeviceType__
