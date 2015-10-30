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
 * \file src/example/exp_09_kuka.cpp
 * 
 */
#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>

#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <mbslib/utility/DeriveOMat.hpp>

#include <math.h>

#define ENDPOINTFORCEID 0
#define ENDPOINTMEASUREID 1
#define ENDPOINTTCPID 2
#define ENDPOINTTOOLID 3

using namespace mbslib;
int EndPointID[4];

DeriveOMat * dom;

MbsCompoundWithBuilder * createRobot(TMatrixX & K, TMatrixX & D, TMatrixX & Map_q, TVectorX & m) {
    // number of joints
    int n = 6;
    // DH - Parameter
    float d[] = {0.75f, 0, 0, -1.1f, 0, -0.23f};
    float a[] = {0.35f, 1.25f, 0, 0, 0, 0};
    float theta[] = {0, -M_PI_2, 0, 0, 0, 0};
    float alpha[] = {-M_PI_2, 0, M_PI_2, -M_PI_2, M_PI_2, M_PI};

    // 1 = rot, 0 = trans
    int h[] = {1, 1, 1, 1, 1, 1};

    // inertia mass tensor
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
    Eigen::MatrixXf s(3, n + 1);
    s << -0.314f, -0.770f, 0, 0, 0, -0.03f, 0,
        0.259f, -0.005f, 0, -0.167f, 0, 0, 0,
        -0.012f, -0.188f, -0.0825f, 0, -0.05f, 0.06f, 0;

    // 1 = tilt axis (virtual)
    Eigen::VectorXi tiltAxesX(n);
    tiltAxesX << 0, 0, 0, 0, 0, 0;
    Eigen::VectorXi tiltAxesY(n);
    tiltAxesY << 0, 0, 0, 0, 0, 0;

    // number of tilt axis
    int nV = tiltAxesX.sum() + tiltAxesY.sum();

    // number of all axis
    int nG = nV + n;

    // gear backslash
    Eigen::VectorXf spiel_q(n);
    spiel_q << 6e-5f, 3.07e-5f, 2.5e-5f, 3e-5f, 1e-5f, 40e-5f;
    spiel_q = spiel_q * M_PI / 180;

    // 1 = use gravity vector in simulation
    Eigen::VectorXf compensation = Eigen::VectorXf::Ones(nG);

    // stiffnes values
    float K_stiff[] = {5.6e+006, 8.0397e+006, 3.1205e+006, 7.1959e+005, 6.5862e+005, 3.1403e+005};
    float K_tiltX[] = {1.33e+007, 2.61e+007, 1.54e+006, 1.93e+006, 6.584e+005, 1.899e+006};
    float K_tiltY[] = {1.33e+007, 2.61e+007, 1.54e+006, 1.93e+006, 6.584e+005, 1.899e+006};

    // damping values
    float D_tiltX[] = {4000, 6000, 3000, 1000, 300, 1000};
    float D_tiltY[] = {4000, 6000, 3000, 1000, 300, 1000};
    float D_stiff[] = {82000, 68000.2542, 5000.1174, 2000.7, 200.436, 300.03};

    // stiffness matrix K
    K = TMatrixX::Zero(nG, nG);
    D = TMatrixX::Zero(nG, nG);

    // Map from real Joints to all axis
    Map_q = TMatrixX::Zero(nG, n);

    int count = 0;
    int q_count = 0;
    for (int i = 0; i < n; i++) {
        K(count, count) = K_stiff[i];
        D(count, count) = D_stiff[i];
        Map_q(q_count, i) = 1;
        q_count++;
        count++;

        if (tiltAxesX[i] == 1) {
            K(count, count) = K_tiltX[i];
            D(count, count) = D_tiltX[i];
            q_count++;
            count++;
        }

        if (tiltAxesY[i] == 1) {
            K(count, count) = K_tiltY[i];
            D(count, count) = D_tiltY[i];
            q_count++;
            count++;
        }
    }

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
            case 0:
                mbs->addRevoluteJoint(axis, jointOffset, "joint0");
                break;
            case 1:
                mbs->addRevoluteJoint(axis, jointOffset, "joint1");
                break;
            case 2:
                mbs->addRevoluteJoint(axis, jointOffset, "joint2");
                break;
            case 3:
                mbs->addRevoluteJoint(axis, jointOffset, "joint3");
                break;
            case 4:
                mbs->addRevoluteJoint(axis, jointOffset, "joint4");
                break;
            case 5:
                mbs->addRevoluteJoint(axis, jointOffset, "joint5");
                break;
            }

            //virtual joints
            if (tiltAxesX[i] == 1) {
                TVector3 axis(1, 0, 0);
                TScalar jointOffset = 0;
                mbs->addRevoluteJoint(axis, jointOffset);
            }
            if (tiltAxesY[i] == 1) {
                TVector3 axis(0, 1, 0);
                TScalar jointOffset = 0;
                mbs->addRevoluteJoint(axis, jointOffset);
            }

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
        } else {
            //prismatic joint
            TMatrix3x3 rotZ;
            rotZ = Eigen::AngleAxis< TScalar >(theta[i], TVector3::UnitZ());
            mbs->addFixedRotation(rotZ);

            TVector3 axis(0, 0, 1);
            TScalar jointOffset = d[i];
            mbs->addPrismaticJoint(axis, jointOffset);

            TMatrix3x3 rotX;
            rotX = Eigen::AngleAxis< TScalar >(alpha[i], TVector3::UnitX());

            TMatrix3x3 ISP_tmp = ISP.at(i);

            TVector3 s_tmp(s(0, i), s(1, i), s(2, i));

            mbs->addRigidLink(TVector3(a[i], 0, 0), rotX * s_tmp, m(i), rotX * ISP_tmp * rotX.transpose());
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
    EndPointID[ENDPOINTFORCEID] = mbs->getNumberOfEndpoints() - 1;

    // Add Measure CoF
    TMatrix3x3 meas_r;
    meas_r = Eigen::AngleAxis< TScalar >(M_PI, TVector3::UnitZ()) * Eigen::AngleAxis< TScalar >(-M_PI_2, TVector3::UnitY()) * Eigen::AngleAxis< TScalar >(0, TVector3::UnitZ());

    TVector3 meas_t(270.0 / 1000, 0.455 / 1000, 120.735 / 1000);

    mbs->addFork();
    mbs->addRigidLink(meas_t, TVector3::Zero(), 0, TMatrix3x3::Zero());
    mbs->addFixedRotation(meas_r);
    mbs->addEndpoint("Measure");
    EndPointID[ENDPOINTMEASUREID] = mbs->getNumberOfEndpoints() - 1;

    // Add Tool CoF
    TMatrix3x3 tool_r;
    tool_r = Eigen::AngleAxis< TScalar >(M_PI, TVector3::UnitZ()) * Eigen::AngleAxis< TScalar >(-M_PI_2, TVector3::UnitY()) * Eigen::AngleAxis< TScalar >(0, TVector3::UnitZ());

    TVector3 tool_t(298.418 / 1000, 0.455 / 1000, 120.735 / 1000);

    mbs->addFork();
    mbs->addRigidLink(tool_t, TVector3::Zero(), 0, TMatrix3x3::Zero());
    mbs->addFixedRotation(tool_r);
    mbs->addEndpoint("Tool");
    EndPointID[ENDPOINTTCPID] = mbs->getNumberOfEndpoints() - 1;

    // Endpoint for TCP

    mbs->addEndpoint("TCP");
    EndPointID[ENDPOINTTOOLID] = mbs->getNumberOfEndpoints() - 1;

    mbs->setGravitation(TVector3(0, 0, -1));
    return mbs;
}

// print robots joint position, velocity and accelration
void printRobotInformation(MbsCompound *& mbs) {
    // Output of EndPoint Position and Orientation
    for (unsigned int i = 0; i < mbs->getNumberOfEndpoints(); i++) {
        std::cout << "\n Endpoint " << i << ":\n"
                  << "r "
                  << "\n"
                  << "X: " << mbs->getEnd(EndPointID[i]).getCoordinateFrame().r.x() << "\n"
                  << "Y: " << mbs->getEnd(EndPointID[i]).getCoordinateFrame().r.y() << "\n"
                  << "Z: " << mbs->getEnd(EndPointID[i]).getCoordinateFrame().r.z() << "\n"
                  << "\nR "
                  << "\n" << mbs->getEnd(EndPointID[i]).getCoordinateFrame().R << std::endl;
    }

    // Output of robots DOF and number of Joints
    std::cout << "DOF: " << mbs->getDOF() << "\n"
              << "Number of Joints: " << mbs->getNumberOfJoints() << std::endl;

    std::cout << "\n" << std::endl;
    // Output of Joint Position
    for (size_t i = 0; i < mbs->getNumberOfJoints(); i++) {
        TScalar q_tmp = mbs->getJoints().at(i)->getJointState().q;
        std::cout << "Joint Position" << i << ": " << q_tmp << std::endl;
    }
    std::cout << "\n" << std::endl;
    // Output of Joint Velocity
    for (size_t i = 0; i < mbs->getNumberOfJoints(); i++) {
        TScalar q_tmp = mbs->getJoints().at(i)->getJointState().dotq;
        std::cout << "Joint Velocity " << i << ": " << q_tmp << std::endl;
    }
    std::cout << "\n" << std::endl;
    // Output of Joint Acceleration
    for (size_t i = 0; i < mbs->getNumberOfJoints(); i++) {
        TScalar q_tmp = mbs->getJoints().at(i)->getJointState().ddotq;
        std::cout << "Joint Acceleration" << i << ": " << q_tmp << std::endl;
    }
    std::cout << "\n" << std::endl;
    // Output of torque
    for (size_t i = 0; i < mbs->getNumberOfJoints(); i++) {
        TScalar q_tmp = mbs->getJoints().at(i)->getJointForceTorque();
        std::cout << "torque " << i << ": " << q_tmp << std::endl;
    }
    std::cout << "\n" << std::endl;
    // Output of ext. torque
    for (size_t i = 0; i < mbs->getNumberOfJoints(); i++) {
        TScalar q_tmp = mbs->getJoints().at(i)->getExternalJointForceTorque();
        std::cout << "Ext. torque " << i << ": " << q_tmp << std::endl;
    }
}

void simulation() {
    // Stiffness matrix
    TMatrixX K;
    //Damping matrix
    TMatrixX D;
    // Map from real Joints to all axis
    TMatrixX Map_q;

    TVectorX pos(6);
    TVectorX vel(6);
    TVectorX m(6);
    TVectorX tau(6);

    // Independents (pos, vel, mass)
    pos << -0.0007375, 0.00004758, -0.029, 0.0001918, -0.001124, 0.004394;

    vel << -0.00163719, 0.000125117, -0.00297498, 0.102322, 0.0303313, -0.650712;

    m << 686.78f, 212, 163, 109, 16.52f, 46;

    MbsCompound * mbs = createRobot(K, D, Map_q, m);
    dom = new DeriveOMat(*mbs);

    dom->addIndependent("joint0", "q");
    dom->addIndependent("joint1", "q");
    dom->addIndependent("joint2", "q");
    dom->addIndependent("joint3", "q");
    dom->addIndependent("joint4", "q");
    dom->addIndependent("joint5", "q");

    dom->addIndependent("joint0", "dq");
    dom->addIndependent("joint1", "dq");
    dom->addIndependent("joint2", "dq");
    dom->addIndependent("joint3", "dq");
    dom->addIndependent("joint4", "dq");
    dom->addIndependent("joint5", "dq");

    dom->addIndependent("link0", "mass");
    dom->addIndependent("link1", "mass");
    dom->addIndependent("link2", "mass");
    dom->addIndependent("link3", "mass");
    dom->addIndependent("link4", "mass");
    dom->addIndependent("link5", "mass");

    dom->addDependent("joint0", "dq");
    dom->addDependent("joint1", "dq");
    dom->addDependent("joint2", "dq");
    dom->addDependent("joint3", "dq");
    dom->addDependent("joint4", "dq");
    dom->addDependent("joint5", "dq");

    dom->addDependent("joint0", "ddq");
    dom->addDependent("joint1", "ddq");
    dom->addDependent("joint2", "ddq");
    dom->addDependent("joint3", "ddq");
    dom->addDependent("joint4", "ddq");
    dom->addDependent("joint5", "ddq");

    double indep[18] = {
        1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1};

    double dep[12];

    tau = K * (-pos) + D * (-vel);
    mbs->setJointPosition(pos);
    mbs->setJointVelocity(vel);
    mbs->setJointForceTorque(tau);

    dom->startTape(indep);
    //mbs->doDirkin();
    // FWDYN
    mbs->doABA();
    dom->endTape(dep);

    printRobotInformation(mbs);

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
    std::cout << "\n" << std::endl;

    std::cout << "\nJacobi:\n";
    for (size_t i = 0; i < 12; i++) {
        for (size_t j = 0; j < 18; j++) {
            std::cout << J[i][j] << " ";
        }
        std::cout << "\n" << std::endl;
    }
    std::cout << "\n" << std::endl;

    delete dom;
    delete mbs;
}

int main(void) {
    simulation();
    return 0;
}
