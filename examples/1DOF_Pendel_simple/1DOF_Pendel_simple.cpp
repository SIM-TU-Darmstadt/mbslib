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

#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>
#include <mbslib/elements/joint/JointForceSetter.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>

using namespace mbslib;

int main(void) {
    /**
     * Set time parameters
    */

    TTime dt = 0.001;
    TTime t = 0;
    TTime duration = 3;

    TScalar g = 9.81;

    /**
     *  Building mbs tree
    */
    MbsCompoundWithBuilder mbs;

    mbs.addFixedBase("base");

    TScalar l_1 = 1;                     //length of first arm
    TScalar m_1 = 1;                     //mass of first arm from dircol documentation, chapter 2.7
    TMatrix3x3 I_1 = TMatrix3x3::Zero(); // inertia of first arm

    // create 1-DOF robot
    Joint1DOF * joint1 = mbs.addRevoluteJoint(TVector3::UnitZ(), 0, 0, 0, 0, "joint1");
    mbs.addRigidLink(TVector3::UnitX() * l_1, TVector3::UnitX() * 0.0 * l_1, m_1, I_1, "arm_1");
    mbs.addEndpoint(0, TMatrix3x3::Zero(), "tool");
    mbs.setGravitation(TVector3(0, -g, 0));

    JointForceSetter * jfs1 = new JointForceSetter(*joint1);
    mbs.addForceGenerator(*jfs1);

    // initilize mbs, starting position
    mbs.doDirkin();

    for (unsigned int i = 0; i < duration / dt - 1; i++) {

        TVectorX actualState = joint1->getState();
        TVectorX actualDState = joint1->getDStateDt();
        TScalar actualControl = joint1->getJointForceTorque();

        joint1->setJointForceTorque(sin(t));
        mbs.doABA();

        // and integrate
        mbs.integrate(dt);
        t += dt;

        TVectorX state = joint1->getState();
        TVectorX dState = joint1->getDStateDt();
        TScalar control = joint1->getJointForceTorque();
        std::cout << "t: " << i << " mbslib: "
                  << " ddq: " << dState[1] << " u: " << control << " anaModell: " << actualControl - g * cos(actualState[0]) << std::endl;
    }

    return 0;
}
