/*
 * Copyright (C) 2015
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
/**
 * \file src/example/exp_03_extforce.cpp
 * 
 */
#include <mbslib/mbslib.hpp>

#include <iostream>
using namespace mbslib;
TScalar toRad(TScalar deg) {
    return M_PI * deg / 180.;
}

int main(void) {
    using namespace mbslib;

    MbsCompoundWithBuilder mbs;

    // Create a simple robot.
    mbs.addFixedBase();
    Joint1DOF * j1 = mbs.addRevoluteJoint(TVector3(0, 0, 1), TScalar(0));
    mbs.addRigidLink(TVector3(1, 0, 0), TVector3(0, 0, 0), 1, TMatrix3x3::Zero());
    Endpoint * tcp = mbs.addEndpoint();

    mbs.doDirkin();

    tcp->setExternalForceTorque(TVector3(0, 0, 0), TVector3(0, 0, 0));
    //std::cout << mbs.calculateExternalForceTorqueInJoints() << std::endl;

    mbs.doCrba();
    std::cout << "q = " << mbs.getJointPosition() << std::endl;
    std::cout << "dq = " << mbs.getJointVelocity() << std::endl;
    std::cout << "tau = " << mbs.getJointForceTorque() << std::endl;
    std::cout << "ddq = " << mbs.getJointAcceleration() << std::endl;

    tcp->setExternalForceTorque(TVector3(0, 1, 0), TVector3(0, 0, 0));
    //std::cout << mbs.calculateExternalForceTorqueInJoints() << std::endl;

    mbs.doCrba();
    std::cout << "q = " << mbs.getJointPosition() << std::endl;
    std::cout << "dq = " << mbs.getJointVelocity() << std::endl;
    std::cout << "tau = " << mbs.getJointForceTorque() << std::endl;
    std::cout << "ddq = " << mbs.getJointAcceleration() << std::endl;

    j1->setJointPosition(1);
    mbs.doCrba();
    std::cout << "q = " << mbs.getJointPosition() << std::endl;
    std::cout << "dq = " << mbs.getJointVelocity() << std::endl;
    std::cout << "tau = " << mbs.getJointForceTorque() << std::endl;
    std::cout << "ddq = " << mbs.getJointAcceleration() << std::endl;
}
