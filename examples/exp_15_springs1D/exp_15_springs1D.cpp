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
 * \file src/example/exp_15_springs1D.cpp
 * 
 */
#include <fstream>
#include <iostream>
#include <math.h>

#include <mbslib/mbslib.hpp>
#include <mbslib/mbslib.muscle.models.hpp>

#include <Eigen/Geometry>

using namespace mbslib;

int main(void) {
    double d[] = {0.75, 0, 0, -1.1, 0, -0.23};
    double a[] = {0.35, 1.25, 0, 0, 0, 0};
    double theta[] = {0, -M_PI_2, 0, 0, 0, 0};
    double alpha[] = {-M_PI_2, 0, M_PI_2, -M_PI_2, M_PI_2, M_PI};

    std::vector< TMatrix3x3 > ISP;
    TMatrix3x3 ISP_tmps;

    ISP_tmps << 72.6632, 0, 0,
        0, 135.1493, 0,
        0, 0, 84.4584;
    ISP.push_back(ISP_tmps);

    ISP_tmps << 4.579, -0.110, 4.667,
        -0.110, 40.562, -0.184,
        4.667, -0.184, 39.952;
    ISP.push_back(ISP_tmps);

    ISP_tmps << 16.958, -0.156, 0.006,
        -0.156, 3.49, -0.975,
        0.006, -0.975, 17.67;
    ISP.push_back(ISP_tmps);

    ISP_tmps << 3.984, 0.000, 0.000,
        0.000, 1.13, -0.116,
        0.000, -0.116, 3.46;
    ISP.push_back(ISP_tmps);

    ISP_tmps << 0.153, 0.000, 0.000,
        0.000, 0.1577, -0.019,
        0.000, -0.019, 0.09;
    ISP.push_back(ISP_tmps);

    ISP_tmps << 0.1885, 0, 0,
        0, 0.244, 0,
        0, 0, 0.217;
    ISP.push_back(ISP_tmps);

    ISP_tmps << 0, 0, 0,
        0, 0, 0,
        0, 0, 0;
    ISP.push_back(ISP_tmps);

    // centre of gravity vectors/ value (n+1) is weight on TCP
    Eigen::MatrixXf s(3, 6 + 1);
    s << -0.314f, -0.770f, 0, 0, 0, -0.03f, 0,
        0.259f, -0.005f, 0, -0.167f, 0, 0, 0,
        -0.012f, -0.188f, -0.0825f, 0, -0.05f, 0.06f, 0;

    MbsCompoundWithBuilder * mbs = new MbsCompoundWithBuilder();
    mbs->addFixedBase();

    std::vector< bool > doDirDyn;

    LinearSpringModel sm1(5.6e+006, 4000);

    TVector3 axis(0, 0, 1);
    TScalar jointOffset = theta[0];

    mbs->addFork();
    Joint1DOF * j1_dirve = mbs->addRevoluteJoint(axis, jointOffset, 0, 0, 0, "j1_drive");
    mbs->addEndpoint();
    Joint1DOF * j1_output = mbs->addRevoluteJoint(axis, jointOffset, 0, 0, 0, "j1_output");

    doDirDyn.push_back(false);
    doDirDyn.push_back(true);

    mbs->addSpring(sm1, *j1_dirve, *j1_output, "spring1");

    TMatrix3x3 rotX;
    rotX = Eigen::AngleAxis< TScalar >(alpha[0], TVector3::UnitX());

    TMatrix3x3 ISP_tmp = ISP.at(0);

    TVector3 s_tmp(s(0, 0), s(1, 0), s(2, 0));

    mbs->addRigidLink(TVector3(a[0], 0, d[0]), rotX * s_tmp, 686.78, rotX * ISP_tmp * rotX.transpose(), "link1");

    mbs->addEndpoint();

    mbs->setGravitation(TVector3(0, 0, -9.81));
    TVectorX q(2);
    q << 0.2, 0.1999999;

    TVectorX dq(2);
    dq << 0.2, 0.2;

    TVectorX ddq(2);
    ddq << 0.1, 0.0;

    TVectorX torque(2);
    torque << 0.0, 0.0;

    mbs->doABAhyb(doDirDyn);

    mbs->setJointPosition(q);
    mbs->setJointVelocity(dq);
    mbs->setJointAcceleration(ddq);
    mbs->setJointForceTorque(torque);

    std::cout << mbs->getJoints().at(0)->getExternalJointForceTorque() << std::endl;
    std::cout << mbs->getJoints().at(1)->getExternalJointForceTorque() << std::endl;

    mbs->doABAhyb(doDirDyn);

    for (size_t i = 0; i < mbs->getNumberOfEndpoints(); ++i) {
        std::cout << "\n Endpoint " << i << ":\n"
                  << "r "
                  << "\n"
                  << "X: " << mbs->getEnd(i).getCoordinateFrame().r.x() << "\n"
                  << "Y: " << mbs->getEnd(i).getCoordinateFrame().r.y() << "\n"
                  << "Z: " << mbs->getEnd(i).getCoordinateFrame().r.z() << "\n"
                  << "\nR "
                  << "\n"
                  << mbs->getEnd(i).getCoordinateFrame().R << std::endl;
    }

    std::cout << "\n"
              << std::endl;
    // Output of Joint Position
    for (size_t i = 0; i < mbs->getNumberOfJoints(); ++i) {
        TScalar q_tmp = mbs->getJoints().at(i)->getJointState().q;
        std::cout << "Joint Position" << i << ": " << q_tmp << std::endl;
    }
    std::cout << "\n"
              << std::endl;
    // Output of Joint Velocity
    for (size_t i = 0; i < mbs->getNumberOfJoints(); ++i) {
        TScalar q_tmp = mbs->getJoints().at(i)->getJointState().dotq;
        std::cout << "Joint Velocity " << i << ": " << q_tmp << std::endl;
    }
    std::cout << "\n"
              << std::endl;
    // Output of Joint Acceleration
    for (size_t i = 0; i < mbs->getNumberOfJoints(); ++i) {
        TScalar q_tmp = mbs->getJoints().at(i)->getJointState().ddotq;
        std::cout << "Joint Acceleration" << i << ": " << q_tmp << std::endl;
    }
    std::cout << "\n"
              << std::endl;
    // Output of tau
    for (size_t i = 0; i < mbs->getNumberOfJoints(); ++i) {
        TScalar q_tmp = mbs->getJoints().at(i)->getJointForceTorque();
        std::cout << "torque " << i << ": " << q_tmp << std::endl;
    }
    std::cout << "\n"
              << std::endl;
    // Output of tau
    for (size_t i = 0; i < mbs->getNumberOfJoints(); ++i) {
        TScalar q_tmp = mbs->getJoints().at(i)->getExternalJointForceTorque();
        std::cout << "external torque " << i << ": " << q_tmp << std::endl;
    }
    delete mbs;
    return 0;
}
