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
 * \file src/example/exp_08_jacobian.cpp
 * 
 */
#include <mbslib/mbslib.hpp>

#include <iostream>

int main(void) {
    using namespace mbslib;

    MbsCompoundWithBuilder mbs;

    FreeBase * fb = mbs.addFreeBase("fb");
    RigidLink * l1 = mbs.addRigidLink(TVector3(1, 0, 0), TVector3::Zero(), 0, TMatrix3x3::Zero(), "l1");
    Fork * f1 = mbs.addFork("f1");

    RevoluteJoint * j1 = mbs.addRevoluteJoint(TVector3(0, 1, 0), 0, "j1");
    RigidLink * l2 = mbs.addRigidLink(TVector3(0, 0, 1), TVector3::Zero(), 0, TMatrix3x3::Zero(), "l2");
    Endpoint * e1 = mbs.addEndpoint("e1");

    RevoluteJoint * j2 = mbs.addRevoluteJoint(TVector3(0, 1, 0), 0, "j2");
    RigidLink * l3 = mbs.addRigidLink(TVector3(0, 0, -1), TVector3::Zero(), 0, TMatrix3x3::Zero(), "l3");
    Endpoint * e2 = mbs.addEndpoint("e2");

    mbs.doDirkin();

    std::cout << mbs.calculateJacobian(*e1) << std::endl
              << std::endl;
    std::cout << mbs.calculateJacobian(*e2) << std::endl
              << std::endl;

    return 0;
}
