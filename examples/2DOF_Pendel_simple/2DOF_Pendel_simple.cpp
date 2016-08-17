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

#include <fstream>
#include <iostream>
#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>
#include <mbslib/elements/joint/JointForceSetter.hpp>
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

    TScalar l_2 = 1;                     //length of first arm
    TScalar m_2 = 1;                     //mass of first arm from dircol documentation, chapter 2.7
    TMatrix3x3 I_2 = TMatrix3x3::Zero(); // inertia of first arm

    // create 2-DOF robot
    Joint1DOF * joint1 = mbs.addRevoluteJoint(TVector3::UnitZ(), 0, 0, 0, 0, "joint1");
    mbs.addRigidLink(TVector3::UnitX() * l_1, TVector3::UnitX() * 0.0 * l_1, m_1, I_1, "arm_1");
    Joint1DOF * joint2 = mbs.addRevoluteJoint(TVector3::UnitZ(), 0, 0, 0, 0, "joint2");
    mbs.addRigidLink(TVector3::UnitX() * l_2, TVector3::UnitX() * 0.0 * l_2, m_2, I_2, "arm_2");
    mbs.addEndpoint(0, TMatrix3x3::Zero(), "tool");
    mbs.setGravitation(TVector3(0, -g, 0));

    JointForceSetter * jfs1 = new JointForceSetter(*joint1);
    mbs.addForceGenerator(*jfs1);

    JointForceSetter * jfs2 = new JointForceSetter(*joint2);
    mbs.addForceGenerator(*jfs2);

    // initilize mbs, starting position
    mbs.doDirkin();

    for (unsigned int i = 0; i < duration / dt - 1; i++) {
        // calculate ddq like in DIRCOL-Model:
        TVectorX actualState = mbs.getState();
        TVectorX actualDState = mbs.getDStateDt();
        TVectorX actualControl = mbs.getJointForceTorque();

        TVectorX ddqDIR(2);
        TMatrixX M(2, 2);
        TScalar Mdet;
        TMatrixX Minv(2, 2);
        TVectorX C(2);
        TVectorX G(2);

        M(1, 1) = m_2 * (pow(l_2, 2));
        M(1, 0) = M(1, 1) + m_2 * (l_1 * l_2 * cos(actualState(2)));
        M(0, 1) = M(1, 0);
        M(0, 0) = M(0, 1) + (m_1 + m_2) * pow(l_2, 2);

        C(1) = m_2 * l_1 * l_2 * sin(actualState(2)) * pow(actualState(1), 2);
        C(0) = C(1) - m_1 * l_1 * l_2 * sin(actualState(2)) * pow((actualState(1) + actualState(3)), 2);

        G(1) = m_2 * g * l_2 * cos(actualState(0) + actualState(2));
        G(0) = G(1) + m_1 * l_1 * g * cos(actualState(0)) + m_2 * l_1 * g * (sin(actualState(2)) * cos(actualState(0) + actualState(2)) + cos(actualState(2)) * sin(actualState(0) + actualState(2)));

        Mdet = M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0);
        if (Mdet != 0) {
            Mdet = 1 / Mdet;
        } else {
            std::cout << "MDet = 0" << std::endl;
        }
        Minv(0) = M(3);
        Minv(3) = M(0);
        Minv(1) = -M(1);
        Minv(2) = -M(2);

        std::cout << " actCon " << actualControl.transpose() << std::endl;
        std::cout << " M " << M << std::endl;
        std::cout << " G " << G.transpose() << std::endl;
        std::cout << " C " << C.transpose() << std::endl;

        ddqDIR = Mdet * Minv * (C + G - actualControl);

        //integrate MBSlib-Model:
        joint1->setJointForceTorque(0);
        joint2->setJointForceTorque(0);
        mbs.doABA();

        // and integrate
        mbs.integrate(dt);
        t += dt;

        TVectorX state = mbs.getState();
        TVectorX dState = mbs.getDStateDt();
        TVectorX control = mbs.getJointForceTorque();
        std::cout << "t: " << i << " mbslib    ddq: " << dState[1] << " " << dState[3] << " u1: " << control[0] << std::endl;
        std::cout << "t: " << i << " anaModell ddq: " << ddqDIR << std::endl;
    }

    return 0;
}
