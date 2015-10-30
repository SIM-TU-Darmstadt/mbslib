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
 * \file src/example/exp_13_springs_2.cpp
 * 
 */
#include <mbslib/mbslib.hpp>

#include <fstream>
#include <iostream>

int main(void) {
    using namespace mbslib;

    MbsCompoundWithBuilder mbs;

    mbs.addFixedBase();
    mbs.addFork();

    PrismaticJoint * pj = mbs.addPrismaticJoint(TVector3(1, 0, 0));
    Endpoint * s1End1 = mbs.addEndpoint();
    mbs.addFixedTranslation(TVector3(1, 0, 0));

    mbs.addFork();
    Endpoint * s1Midpoint = mbs.addEndpoint();

    mbs.addFixedTranslation(TVector3(1, 0, 0));

    RevoluteJoint * rj = mbs.addRevoluteJoint(TVector3(0, 0, 1));

    mbs.addFork();
    mbs.addFixedTranslation(TVector3(0, 1, 0));
    Endpoint * s1End2 = mbs.addEndpoint();
    mbs.addRigidLink(TVector3(1, 0, 0), TVector3::Zero(), 1, TMatrix3x3::Zero());
    Endpoint * tcp = mbs.addEndpoint();

    LinearSpringWithRopeModel springModel(1, 0, 1);
    Spring3D * spring = mbs.addSpring(springModel);
    (*spring) << s1End1 << s1Midpoint << s1End2;

    //mbs.doDirkin();

    //spring.resetForce();
    //spring.applyForce();

    mbs.doRne();

    std::cout << s1End1->getCoordinateFrame().r.transpose() << std::endl;
    std::cout << s1End2->getCoordinateFrame().r.transpose() << std::endl;
    std::cout << tcp->getCoordinateFrame().r.transpose() << std::endl;
    std::cout << spring->getSpringForce() << std::endl;
    std::cout << pj->getJointForceTorque() << std::endl;
    std::cout << rj->getJointForceTorque() << std::endl;
}
