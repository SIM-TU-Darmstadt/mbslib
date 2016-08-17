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

#ifndef __MBSLIB_LBR2MODEL_HPP__
#define __MBSLIB_LBR2MODEL_HPP__

#include <mbslib/utility/types.hpp>

namespace mbslib {
class LBR2Model {
public:
    typedef TVectorX JointPositions;
    typedef TVectorX JointVelocities;
    typedef TVectorX JointAccelerations;
    typedef TVectorX JointTorques;
    typedef TVector3 Vector3;
    typedef TMatrixX Matrix;

public:
    virtual ~LBR2Model();
    virtual JointAccelerations forwardDynamics(const JointPositions & q, const JointVelocities & qdot, const JointTorques & tau) = 0;
    virtual JointTorques inverseDynamics(const JointPositions & q, const JointVelocities & qdot, const JointAccelerations & qdotdot) = 0;
    virtual JointTorques gravityTorque(const JointPositions & q) = 0;
    virtual JointTorques coriolisTorque(const JointPositions & q, const JointVelocities & qdot) = 0;
    virtual JointTorques massTorque(const JointPositions & q, const JointAccelerations & qdotdot) = 0;
    virtual Matrix massMatrix(const JointPositions & q) = 0;
    virtual Vector3 forwardKinematics(const JointPositions & q) = 0;
};
}

#endif //__MBSLIB_LBR2MODEL_HPP__
