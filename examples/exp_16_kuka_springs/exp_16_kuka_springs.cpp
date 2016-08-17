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
 * \file src/example/exp_16_kuka_springs.cpp
 * 
 */
#include <fstream>
#include <iostream>
#include <math.h>

#include <Eigen/Geometry>
#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>
#include <mbslib/elements/spring/model/LinearSpringModel.hpp>
#include <mbslib/utility/DeriveOMat.hpp>

using namespace mbslib;
std::vector< int > Endpoints;
DeriveOMat * dom;

MbsCompoundWithBuilder * createRobot(TVectorX & K, TVectorX & D, TVectorX & m, std::vector< bool > & doDirDyn) {
    int n = 6;
    float d[] = {0.75f, 0, 0, -1.1f, 0, -0.23f};
    float a[] = {0.35f, 1.25f, 0, 0, 0, 0};
    float theta[] = {0, -M_PI_2, 0, 0, 0, 0};
    float alpha[] = {-M_PI_2, 0, M_PI_2, -M_PI_2, M_PI_2, M_PI};

    // 1 = rot, 0 = trans
    int h[] = {1, 1, 1, 1, 1, 1};

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

    // build mbs with robot parameters
    for (int i = 0; i < n; i++) {
        if (h[i] == 1) {
            //revolute joint
            if (i == 3 || i == 5) {
                TVector3 relr(0, 0, d[i]);
                TVector3 com(0, 0, 0);
                TScalar m = 0;
                TMatrix3x3 I = TMatrix3x3::Zero();
                mbs->addRigidLink(relr, com, m, I);
            }
            TVector3 axis(0, 0, 1);
            TScalar jointOffset = theta[i];

            switch (i) {
            case 0: {
                mbs->addFork();
                Joint1DOF * j0_drive = mbs->addRevoluteJoint(axis, jointOffset, "j0_drive");
                mbs->addEndpoint();
                Joint1DOF * j0_output = mbs->addRevoluteJoint(axis, jointOffset, "j0_output");
                LinearSpringModel sm0(K(i), D(i));
                mbs->addSpring(sm0, *j0_drive, *j0_output, "spring0");
                break;
            }
            case 1: {
                mbs->addFork();
                Joint1DOF * j1_drive = mbs->addRevoluteJoint(axis, jointOffset, "j1_drive");
                mbs->addEndpoint();
                Joint1DOF * j1_output = mbs->addRevoluteJoint(axis, jointOffset, "j1_output");
                LinearSpringModel sm1(K(i), D(i));
                mbs->addSpring(sm1, *j1_drive, *j1_output, "spring1");
                break;
            }
            case 2: {
                mbs->addFork();
                Joint1DOF * j2_drive = mbs->addRevoluteJoint(axis, jointOffset, "j2_drive");
                mbs->addEndpoint();
                Joint1DOF * j2_output = mbs->addRevoluteJoint(axis, jointOffset, "j2_output");
                LinearSpringModel sm2(K(i), D(i));
                mbs->addSpring(sm2, *j2_drive, *j2_output, "spring2");
                break;
            }
            case 3: {
                mbs->addFork();
                Joint1DOF * j3_drive = mbs->addRevoluteJoint(axis, jointOffset, "j3_drive");
                mbs->addEndpoint();
                Joint1DOF * j3_output = mbs->addRevoluteJoint(axis, jointOffset, "j3_output");
                LinearSpringModel sm3(K(i), D(i));
                mbs->addSpring(sm3, *j3_drive, *j3_output, "spring3");
                break;
            }

            case 4: {
                mbs->addFork();
                Joint1DOF * j4_drive = mbs->addRevoluteJoint(axis, jointOffset, "j4_drive");
                mbs->addEndpoint();
                Joint1DOF * j4_output = mbs->addRevoluteJoint(axis, jointOffset, "j4_output");
                LinearSpringModel sm4(K(i), D(i));
                mbs->addSpring(sm4, *j4_drive, *j4_output, "spring4");
                break;
            }

            case 5: {
                mbs->addFork();
                Joint1DOF * j5_drive = mbs->addRevoluteJoint(axis, jointOffset, "j5_drive");
                mbs->addEndpoint();
                Joint1DOF * j5_output = mbs->addRevoluteJoint(axis, jointOffset, "j5_output");
                LinearSpringModel sm5(K(i), D(i));
                mbs->addSpring(sm5, *j5_drive, *j5_output, "spring5");
                break;
            }
            }
            doDirDyn.push_back(false); // InvDyn for drive
            doDirDyn.push_back(true);  // DirDyn for output

            TMatrix3x3 rotX;
            rotX = Eigen::AngleAxis< TScalar >(alpha[i], TVector3::UnitX());

            TMatrix3x3 ISP_tmp = ISP.at(i);

            TVector3 s_tmp(s(0, i), s(1, i), s(2, i));

            switch (i) {
            case 0:
                mbs->addRigidLink(TVector3(a[0], 0, d[0]), rotX * s_tmp, m(0), rotX * ISP_tmp * rotX.transpose(), "link0");
                break;
            case 1:
                mbs->addRigidLink(TVector3(a[1], 0, d[1]), rotX * s_tmp, m(1), rotX * ISP_tmp * rotX.transpose(), "link1");
                break;
            case 2:
                mbs->addRigidLink(TVector3(a[2], 0, d[2]), rotX * s_tmp, m(2), rotX * ISP_tmp * rotX.transpose(), "link2");
                break;
            case 3:
                mbs->addRigidLink(TVector3(a[3], 0, 0), rotX * s_tmp, m(3), rotX * ISP_tmp * rotX.transpose(), "link3");
                break;
            case 4:
                mbs->addRigidLink(TVector3(a[4], 0, d[4]), rotX * s_tmp, m(4), rotX * ISP_tmp * rotX.transpose(), "link4");
                break;
            case 5:
                mbs->addRigidLink(TVector3(a[5], 0, 0), rotX * s_tmp, m(5), rotX * ISP_tmp * rotX.transpose(), "link5");
                break;
            }

            mbs->addFixedRotation(rotX);
        }
    }

    // special case for kuka robot
    TMatrix3x3 rotZ;
    rotZ = Eigen::AngleAxis< TScalar >(M_PI, TVector3::UnitZ());

    mbs->addFixedRotation(rotZ);

    // rigid links for TCP mass / CoM / inertia
    TMatrix3x3 ISP_tmp = ISP.at(n);

    TVector3 s_tmp(s(0, n), s(1, n), s(2, n));

    mbs->addRigidLink(TVector3::Zero(), s_tmp, 0, ISP_tmp);

    // Add Force CoF
    TMatrix3x3 force_r;
    force_r = Eigen::AngleAxis< TScalar >(M_PI, TVector3::UnitZ()) * Eigen::AngleAxis< TScalar >(-M_PI_2, TVector3::UnitY()) * Eigen::AngleAxis< TScalar >(0, TVector3::UnitZ());

    TVector3 force_t(220.0 / 1000, 0.455 / 1000, 120.735 / 1000);

    mbs->addFork();
    mbs->addRigidLink(force_t, TVector3::Zero(), 0, TMatrix3x3::Zero());
    mbs->addFixedRotation(force_r);
    mbs->addEndpoint("Force");
    //EndPointID[ENDPOINTFORCEID] = mbs->getNumberOfEndpoints()-1;

    // Add Measure CoF
    TMatrix3x3 meas_r;
    meas_r = Eigen::AngleAxis< TScalar >(M_PI, TVector3::UnitZ()) * Eigen::AngleAxis< TScalar >(-M_PI_2, TVector3::UnitY()) * Eigen::AngleAxis< TScalar >(0, TVector3::UnitZ());

    TVector3 meas_t(270.0 / 1000, 0.455 / 1000, 120.735 / 1000);

    mbs->addFork();
    mbs->addRigidLink(meas_t, TVector3::Zero(), 0, TMatrix3x3::Zero());
    mbs->addFixedRotation(meas_r);
    mbs->addEndpoint("Measure");
    //EndPointID[ENDPOINTMEASUREID] = mbs->getNumberOfEndpoints()-1;

    // Add Tool CoF
    TMatrix3x3 tool_r;
    tool_r = Eigen::AngleAxis< TScalar >(M_PI, TVector3::UnitZ()) * Eigen::AngleAxis< TScalar >(-M_PI_2, TVector3::UnitY()) * Eigen::AngleAxis< TScalar >(0, TVector3::UnitZ());

    TVector3 tool_t(298.418 / 1000, 0.455 / 1000, 120.735 / 1000);

    mbs->addFork();
    mbs->addRigidLink(tool_t, TVector3::Zero(), 0, TMatrix3x3::Zero());
    mbs->addFixedRotation(tool_r);
    mbs->addEndpoint("Tool");
    //EndPointID[ENDPOINTTCPID] = mbs->getNumberOfEndpoints()-1;

    // Endpoint for TCP
    mbs->addEndpoint("TCP");
    //  EndPointID[ENDPOINTTOOLID] = mbs->getNumberOfEndpoints()-1;

    mbs->setGravitation(TVector3(0, 0, -1));

    return mbs;
}

void setupTape() {

    TVectorX K(6);
    TVectorX D(6);
    TVectorX m(6);
    K << 5.6e+006, 8.0397e+006, 3.1205e+006, 7.1959e+005, 6.5862e+005, 3.1403e+005;
    D << 82000, 68000.2542, 5000.1174, 2000.7, 200.436, 300.03;
    m << 686.78f, 212, 163, 109, 16.52f, 46;

    std::vector< bool > doDirDyn;

    MbsCompound * mbs = createRobot(K, D, m, doDirDyn);
    dom = new DeriveOMat(*mbs);

    dom->addIndependent("j0_output", "q");
    dom->addIndependent("j1_output", "q");
    dom->addIndependent("j2_output", "q");
    dom->addIndependent("j3_output", "q");
    dom->addIndependent("j4_output", "q");
    dom->addIndependent("j5_output", "q");

    dom->addIndependent("j0_output", "dq");
    dom->addIndependent("j1_output", "dq");
    dom->addIndependent("j2_output", "dq");
    dom->addIndependent("j3_output", "dq");
    dom->addIndependent("j4_output", "dq");
    dom->addIndependent("j5_output", "dq");

    dom->addIndependent("link0", "mass");
    dom->addIndependent("link1", "mass");
    dom->addIndependent("link2", "mass");
    dom->addIndependent("link3", "mass");
    dom->addIndependent("link4", "mass");
    dom->addIndependent("link5", "mass");
    //  dom->addIndependent("spring0", "springconstant");
    //  dom->addIndependent("spring1", "springconstant");
    //  dom->addIndependent("spring2", "springconstant");
    //  dom->addIndependent("spring3", "springconstant");
    //  dom->addIndependent("spring4", "springconstant");
    //  dom->addIndependent("spring5", "springconstant");

    dom->addDependent("j0_output", "dq");
    dom->addDependent("j1_output", "dq");
    dom->addDependent("j2_output", "dq");
    dom->addDependent("j3_output", "dq");
    dom->addDependent("j4_output", "dq");
    dom->addDependent("j5_output", "dq");

    dom->addDependent("j0_output", "ddq");
    dom->addDependent("j1_output", "ddq");
    dom->addDependent("j2_output", "ddq");
    dom->addDependent("j3_output", "ddq");
    dom->addDependent("j4_output", "ddq");
    dom->addDependent("j5_output", "ddq");

    double indep[18] = {
        1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1};

    double dep[12];

    TVectorX q_soll(6);
    TVectorX dq_soll(6);
    TVectorX q_ist(6);
    TVectorX dq_ist(6);
    q_soll << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    dq_soll << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    q_ist << -0.0007375, 0.00004758, -0.029, 0.0001918, -0.001124, 0.004394;
    dq_ist << -0.00163719, 0.000125117, -0.00297498, 0.102322, 0.0303313, -0.650712;

    TVectorX q(12);
    TVectorX dq(12);
    q << q_soll(0), q_ist(0), q_soll(1), q_ist(1), q_soll(2), q_ist(2), q_soll(3), q_ist(3), q_soll(4), q_ist(4), q_soll(5), q_ist(5);
    dq << dq_soll(0), dq_ist(0), dq_soll(1), dq_ist(1), dq_soll(2), dq_ist(2), dq_soll(3), dq_ist(3), dq_soll(4), dq_ist(4), dq_soll(5), dq_ist(5);

    mbs->setJointPosition(q);
    mbs->setJointVelocity(dq);

    dom->startTape(indep);

    mbs->doABAhyb(doDirDyn);

    dom->endTape(dep);

    std::cout << "\n"
              << std::endl;
    // Output of Joint Position
    for (size_t i = 0; i < mbs->getNumberOfJoints(); i++) {
        TScalar q_tmp = mbs->getJoints().at(i)->getJointState().q;
        std::cout << "Joint Position" << i << ": " << q_tmp << std::endl;
    }
    std::cout << "\n"
              << std::endl;
    // Output of Joint Velocity
    for (size_t i = 0; i < mbs->getNumberOfJoints(); i++) {
        TScalar q_tmp = mbs->getJoints().at(i)->getJointState().dotq;
        std::cout << "Joint Velocity " << i << ": " << q_tmp << std::endl;
    }
    std::cout << "\n"
              << std::endl;
    // Output of Joint Acceleration
    for (size_t i = 0; i < mbs->getNumberOfJoints(); i++) {
        TScalar q_tmp = mbs->getJoints().at(i)->getJointState().ddotq;
        std::cout << "Joint Acceleration" << i << ": " << q_tmp << std::endl;
    }
    std::cout << "\n"
              << std::endl;
    // Output of torque
    for (size_t i = 0; i < mbs->getNumberOfJoints(); i++) {
        TScalar q_tmp = mbs->getJoints().at(i)->getJointForceTorque();
        std::cout << "torque " << i << ": " << q_tmp << std::endl;
    }
    std::cout << "\n"
              << std::endl;
    // Output of ext. torque
    for (size_t i = 0; i < mbs->getNumberOfJoints(); i++) {
        TScalar q_tmp = mbs->getJoints().at(i)->getExternalJointForceTorque();
        std::cout << "Ext. torque " << i << ": " << q_tmp << std::endl;
    }

    delete mbs;
}

int main() {
    setupTape();

    double indep[18]; /* = {
    1,1,1,1,1,1,
    1,1,1,1,1,1,
    1,1,1,1,1,1 }; */

    indep[0] = M_PI_4;
    indep[1] = M_PI_4;
    indep[2] = M_PI_4;
    indep[3] = M_PI_4;
    indep[4] = M_PI_4;
    indep[5] = M_PI_4;
    indep[6] = -0.00163719;
    indep[7] = 0.000125117;
    indep[8] = -0.00297498;
    indep[9] = 0.102322;
    indep[10] = 0.0303313;
    indep[11] = -0.650712;
    indep[12] = 686.78;
    indep[13] = 212;
    indep[14] = 163;
    indep[15] = 109;
    indep[16] = 16.52;
    indep[17] = 46;

    const double * value;
    const double * const * J;

    value = dom->evaluateFunction(indep);
    J = dom->calculateJacobian(indep);

    std::cout << "\nFunction:\n";
    for (size_t i = 0; i < 12; i++) {
        std::cout << value[i] << " ";
    }
    std::cout << "\n"
              << std::endl;

    std::cout << "\nJacobi:\n";
    for (size_t i = 0; i < 12; i++) {
        for (size_t j = 0; j < 18; j++) {
            std::cout << J[i][j] << " ";
        }
        std::cout << "\n"
                  << std::endl;
    }
    std::cout << "\n"
              << std::endl;

    delete dom;
    return 0;
}
