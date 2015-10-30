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
 * \file src/example/exp_11_simulation.cpp
 * 
 */
#include <mbslib/mbslib.hpp>
#include <mbslib/mbslib.muscle.models.hpp>

#include <iostream>

int main(void) {
    using namespace mbslib;

    MbsCompoundWithBuilder mbs;

    FixedBase * fb = mbs.addFixedBase("fb");

    PrismaticJoint * j1 = mbs.addPrismaticJoint(TVector3(0, 0, -1), 0, "j1");
    RigidLink * l1 = mbs.addRigidLink(TVector3(0, 0, 0), TVector3::Zero(), 1, TMatrix3x3::Identity(), "l1");

    PrismaticJoint * j2 = mbs.addPrismaticJoint(TVector3(0, 0, -1), 0, "j2");
    //RevoluteJoint * j2 = mbs.addRevoluteJoint(TVector3(1,0,0),0,"j2");
    RigidLink * l2 = mbs.addRigidLink(TVector3(0, 0, 0), TVector3::Zero(), 1, TMatrix3x3::Identity(), "l2");

    //mbs.addFork();

    Endpoint * e1 = mbs.addEndpoint("e1");

    mbs.setGravitation(TVector3(0, 0, -1));

    //mbs.addRigidLink(TVector3(0,1,0),TVector3::Zero(),0,TMatrix3x3::Zero(),"l3");
    //Endpoint *       e2 = mbs.addEndpoint("e2");

    j1->getJointState().dotq = 1;
    j2->getJointState().dotq = 0;

    CollisionDetector cd(TVector3(0, 0, -1), TVector3(0, 0, 1));
    cd.addEndpoint(*e1);

    SimulationControl sc(mbs, cd);

    TTime t = 0;
    TTime dt = 0.1;
    for (int i = 0; i < 20; i++) {
        std::cout << " t = " << t << " e1.r = " << e1->getCoordinateFrame().r.transpose() << " q = " << mbs.getJointPosition().transpose() << " dq = " << mbs.getJointVelocity().transpose() << " collisions: " << cd.getContacts().size() << std::endl;
        t += dt;
        sc.doTimestep(dt, 0.01);
    }

    return 0;
}
