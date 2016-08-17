/*
 * Copyright (C) 2016
 * Simulation, Systems Optimization and Robotics Group (SIM)
 * Technische Universitaet Darmstadt
 * Hochschulstr. 10
 * 64289 Darmstadt, Germany
 * www.sim.tu-darmstadt.de
 *
 * This file is part of the MBSlib.
 * All rights are reserved by the copyright holder.
 *
 * MBSlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation in version 3 of the License.
 *
 * The MBSlib is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with MBSlib.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __MBSLIB_LBR2MODELANALYTIC_HPP__
#define __MBSLIB_LBR2MODELANALYTIC_HPP__

#include <mbslib/mbslib.hpp>
#include <model.lbr2/LBR2Model.hpp>
#include <model.lbr2/LBR2ModelParameters.h>
#include <vector>

namespace mbslib {
class LBR2ModelAnalytic : public LBR2Model {

public:
    LBR2ModelAnalytic();
    virtual ~LBR2ModelAnalytic();
    virtual JointAccelerations forwardDynamics(const JointPositions & q, const JointVelocities & qdot, const JointTorques & tau);
    virtual JointTorques inverseDynamics(const JointPositions & q, const JointVelocities & qdot, const JointAccelerations & qdotdot);
    virtual JointTorques gravityTorque(const JointPositions & q);
    virtual JointTorques coriolisTorque(const JointPositions & q, const JointVelocities & qdot);
    virtual JointTorques massTorque(const JointPositions & q, const JointAccelerations & qdotdot);
    virtual Matrix massMatrix(const JointPositions & q);
    virtual Vector3 forwardKinematics(const JointPositions & q);
};
}

#endif //__MBSLIB_LBR2MODELANALYTIC_HPP__
