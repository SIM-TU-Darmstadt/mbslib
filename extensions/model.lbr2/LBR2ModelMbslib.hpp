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

#ifndef __MBSLIB_LBR2MODELMBSLIB_HPP__
#define __MBSLIB_LBR2MODELMBSLIB_HPP__

#include <mbslib/mbslib.hpp>
#include <model.lbr2/LBR2Model.hpp>
#include <model.lbr2/LBR2ModelParameters.h>
#include <vector>

namespace mbslib {
class LBR2ModelMbslib : public LBR2Model {
public:
    // enum of algorithms
    enum Algorithm {
        ABA,
        CRBA
    };

    enum class Model {
        ModelDH1,
        ModelDH2
    };

    // field indicating which algorithm to use
    Algorithm algorithm;

    // default constructor sets algorithm to CRBA
    LBR2ModelMbslib(const Model model = Model::ModelDH1);
    virtual ~LBR2ModelMbslib();

    JointAccelerations forwardDynamics(const JointPositions & q, const JointVelocities & qdot, const JointTorques & tau);
    JointTorques inverseDynamics(const JointPositions & q, const JointVelocities & qdot, const JointAccelerations & qdotdot);

    virtual JointTorques gravityTorque(const JointPositions & q);
    virtual JointTorques coriolisTorque(const JointPositions & q, const JointVelocities & qdot);
    virtual JointTorques massTorque(const JointPositions & q, const JointAccelerations & qdotdot);
    virtual Matrix massMatrix(const JointPositions & q);

    virtual Vector3 forwardKinematics(const JointPositions & q);

    // vector field containing joint pointers
    std::vector< RevoluteJoint * > joints;

    MbsCompound * getCompound() {
        return mbs;
    }

private:
    // pointer to model
    MbsCompoundWithBuilder * mbs;

    // helper method builds model
    void buildModel();

    Endpoint * load;
    std::vector< RigidLink * > arms;

    Real mL;
    Vector3 rL;
    const size_t dof;

protected:
    LBR2ModelParameters armParameters;
    Model model;
};
}

#endif // __MBSLIB_LBR2MODELMBSLIB_HPP__
