/**
 **********************************************************************************************************************
 * @file       HaplyFourDoFMech.h
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V0.1.0
 * @date       14-December-2017
 * @brief      Haply 4-DOF Mechanism (TODO)
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#ifndef __HaplyHaplyFourDoFMech__
#define __HaplyHaplyFourDoFMech__

#include "Haply/Mechanisms.h"

namespace Haply
{

class HaplyFourDoFMech : public Mechanisms
{

public:
    HaplyFourDoFMech();

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

#endif // __HaplyHaplyFourDoFMech__
