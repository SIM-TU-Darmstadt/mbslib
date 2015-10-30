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
 * \file src/example/exp_04_dirkin.cpp
 * 
 */
#include <mbslib/mbslib.hpp>

#include <iostream>
using namespace mbslib;
TVector3 testfunction(TScalar l1, TScalar l2, TScalar q1, TScalar q2) {
    using namespace mbslib;

    MbsCompoundWithBuilder mbs;

    mbs.addFixedBase();
    Joint1DOF & j1 = *mbs.addRevoluteJoint(TVector3::UnitZ());
    mbs.addRigidLink(TVector3(l1, 0, 0), TVector3::Zero(), TScalar(1), TMatrix3x3::Zero());
    Joint1DOF & j2 = *mbs.addRevoluteJoint(TVector3::UnitZ());
    mbs.addRigidLink(TVector3(l2, 0, 0), TVector3::Zero(), TScalar(1), TMatrix3x3::Zero());
    Endpoint * tcp = mbs.addEndpoint();

    j1.setJointPosition(q1);
    j2.setJointPosition(q2);

    mbs.doDirkin();

    TMatrix6xX J = mbs.calculateJacobian(*tcp);

    std::cout << J << std::endl;

    return tcp->getCoordinateFrame().r;
}

int main(void) {

    TScalar pi = 3.141592;

    std::cout << testfunction(1, 1, 0, 0) << std::endl
              << std::endl;
    std::cout << testfunction(1, 2, 0, 0) << std::endl
              << std::endl;
    std::cout << testfunction(1, 1, pi / 2, 0) << std::endl
              << std::endl;
    std::cout << testfunction(1, 1, 0, pi / 2) << std::endl
              << std::endl;

    return 0;
}
