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
 * \file src/example/exp_12_springs.cpp
 * 
 */

#include <mbslib/mbslib.hpp>
#include <mbslib/mbslib.muscle.models.hpp>

#include <fstream>
#include <iostream>

int main(void) {
    std::ofstream f("springlog.txt");

    using namespace mbslib;

    MbsCompoundWithBuilder mbs;

    mbs.setGravitation(TVector3(0, 0, 0));

    FixedBase * base = mbs.addFixedBase();
    mbs.addFork();
    Joint1DOF * j3 = mbs.addPrismaticJoint(TVector3(0, 0, 1));
    Joint1DOF * j4 = mbs.addPrismaticJoint(TVector3(0, 1, 0));
    //mbs.addRigidLink(TVector3::Zero(),TVector3::Zero(),1,TMatrix3x3::Zero());
    Endpoint * ep1 = mbs.addEndpoint(1, TMatrix3x3::Zero(), "ep1");

    Joint1DOF * j1 = mbs.addPrismaticJoint(TVector3(0, 0, 1));
    Joint1DOF * j2 = mbs.addPrismaticJoint(TVector3(0, 1, 0));
    //mbs.addRigidLink(TVector3::Zero(),TVector3::Zero(),1,TMatrix3x3::Zero());
    Endpoint * ep2 = mbs.addEndpoint(1, TMatrix3x3::Zero(), "ep2");

    mbs.doDirkin();
    std::cout << mbs.calculateMassMatrix() << std::endl
              << std::endl;

    std::cout << mbs.calculateMassMatrix2() << std::endl;

    CollisionDetector cd(TVector3(0, 0, -1), TVector3(0, 0, 1));

    LinearSpringWithRopeModel springModel(100, 0.5, 1);
    (*(mbs.addSpring(springModel))) << ep2 << ep1;

    SimulationControl simulation(mbs, cd /*,efs*/);

    j1->getJointState().q = 1;
    j2->getJointState().dotq = 1;
    j4->getJointState().dotq = 0;

    for (int i = 0; i < 10000; i++) {
        //std::cout << "t = " << simulation.getTime() << " q = " << j1->getJointState().q << std::endl;
        f << j1->getJointState().q << " " << j2->getJointState().q << " " << j3->getJointState().q << " " << j4->getJointState().q << std::endl;
        simulation.doTimestep(0.001, 1000);
    }
}