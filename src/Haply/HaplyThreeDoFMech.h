/**
 **********************************************************************************************************************
 * @file       HaplyThreeDoFMech.h
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V0.1.0
 * @date       14-December-2017
 * @brief      Haply 3-DOF Mechanism (TODO)
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#ifndef __HaplyHaplyThreeDoFMech__
#define __HaplyHaplyThreeDoFMech__

#include "Haply/Mechanisms.h"

namespace Haply
{

class HaplyThreeDoFMech: public Mechanisms
{

public:
    HaplyThreeDoFMech();

    virtual void forwardKinematics(std::vector<float> angles);
    virtual void torqueCalculation(std::vector<float> force);
    virtual void forceCalculation();
    virtual void positionControl();
    virtual void inverseKinematics();
    virtual void set_mechanism_parameters(std::vector<float> parameters);
    virtual std::vector<float> get_coordinate();
    virtual std::vector<float> get_torque();
    virtual std::vector<float> get_angle();
};

}

#endif // __HaplyHaplyThreeDoFMech__
