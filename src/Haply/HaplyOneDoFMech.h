/**
 **********************************************************************************************************************
 * @file       HaplyOneDoFMech.h
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V0.1.0
 * @date       14-December-2017
 * @brief      Haply 1-DOF Mechanism (TODO)
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#ifndef __HaplyHaplyOneDoFMech__
#define __HaplyHaplyOneDoFMech__

#include "Haply/Mechanisms.h"

namespace Haply
{

class HaplyOneDoFMech: public Mechanisms
{

public:
    HaplyOneDoFMech();

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

} // namespace Haply

#endif // __HaplyHaplyOneDoFMech__
