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

#include <model.lbr2/LBR2ModelMbslib.hpp>
#include <model.lbr2/LBR2ModelParameters.h>

#include <Eigen/Dense>
#include <iostream>
#include <sstream>

using namespace mbslib;

auto LBR2ModelMbslib::forwardDynamics(const JointPositions & q, const JointVelocities & qdot, const JointTorques & tau) -> JointAccelerations {
    JointAccelerations qdotdot(dof);

    mbs->setJointPosition(q);
    mbs->setJointVelocity(qdot);
    mbs->setJointForceTorque(tau);

    switch (algorithm) {
    case ABA:
        mbs->doABA();
        qdotdot = mbs->getJointAcceleration();
        break;
    case CRBA:
        mbs->doCrba();
        qdotdot = mbs->getJointAcceleration();
        break;
    }

    return qdotdot;
}

auto LBR2ModelMbslib::inverseDynamics(const JointPositions & q, const JointVelocities & qdot, const JointAccelerations & qdotdot) -> JointTorques {
    JointTorques tau;

    mbs->setJointPosition(q);
    mbs->setJointVelocity(qdot);
    mbs->setJointAcceleration(qdotdot);

    mbs->doRne();

    tau = mbs->getJointForceTorque();

    return tau;
}

LBR2ModelMbslib::Matrix LBR2ModelMbslib::massMatrix(const JointPositions & q) {
    mbs->setJointPosition(q);
    return mbs->calculateMassMatrix2();
}

LBR2ModelMbslib::JointTorques LBR2ModelMbslib::gravityTorque(const JointPositions & q) {
    JointTorques tau;
    mbs->setJointPosition(q);
    tau = mbs->calculateGravitationVectorInJoints();
    return tau;
}

LBR2ModelMbslib::JointTorques LBR2ModelMbslib::coriolisTorque(const JointPositions & q, const JointVelocities & qdot) {
    JointTorques tau;
    mbs->setJointPosition(q);
    mbs->setJointVelocity(qdot);
    tau = mbs->calculateCoriolisForceInJoints();
    return tau;
}

LBR2ModelMbslib::JointTorques LBR2ModelMbslib::massTorque(const JointPositions & q, const JointAccelerations & qdotdot) {
    JointTorques tau;
    mbs->setJointPosition(q);
    tau = mbs->calculateMassMatrix() * qdotdot;
    return tau;
}

void LBR2ModelMbslib::buildModel() {
    mbs = new MbsCompoundWithBuilder();
    mbs->addFixedBase();

    LBR2Arm * arm = armParameters.arm.data();

    TVector3 e1, e2, e3;
    e1 = TVector3::UnitX();
    e2 = TVector3::UnitY();
    e3 = TVector3::UnitZ();

    for (int i = 0; i < dof; i++) {
        std::stringstream ss;
        ss << "q" << (i + 1);
        switch (model) {
        case Model::ModelDH1:
            mbs->addDHJointLink(arm[i].jointType, arm[i].theta, arm[i].d, arm[i].a, arm[i].alpha, arm[i].com, arm[i].m, arm[i].I, ss.str());
            break;
        case Model::ModelDH2:
        default:
            mbs->addDHJointLink2(arm[i].jointType, arm[i].theta, arm[i].d, arm[i].a, arm[i].alpha, arm[i].com, arm[i].m, arm[i].I, ss.str());
            break;
        }
    }

    mbs->addFixedTranslation(rL, "rL");
    load = mbs->addEndpoint(mL, TMatrix3x3::Zero(), "load");

    TVector3 g(0, 0, -9.81);
    mbs->setGravitation(g);
}

LBR2ModelMbslib::Vector3 LBR2ModelMbslib::forwardKinematics(const JointPositions & q) {
    mbs->setJointPosition(q);
    mbs->doDirkin();
    return mbs->getEnd().getCoordinateFrame().r;
}

LBR2ModelMbslib::LBR2ModelMbslib(Model model)
    : algorithm(CRBA)
    , mbs(nullptr)
    , load(nullptr)
    , mL(0.0)
    , dof(7)
    , armParameters(mbslib::getLBR2ModelParameters())
    , model(model) {
    rL.setZero();
    buildModel();
}

LBR2ModelMbslib::~LBR2ModelMbslib() {
    delete mbs;
}
