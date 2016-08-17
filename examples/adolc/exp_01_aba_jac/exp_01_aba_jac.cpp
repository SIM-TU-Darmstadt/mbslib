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
 * \file src/example/exp_01.cpp
 * 
 */
#include <mbslib/elements/base/FixedBase.hpp>
#include <mbslib/elements/endpoint/Endpoint.hpp>
#include <mbslib/elements/joint/PrismaticJoint.hpp>
#include <mbslib/elements/joint/RevoluteJoint.hpp>
#include <mbslib/elements/rigidbody/RigidLink.hpp>

#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>

#include <iostream>

#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>
#include <mbslib/elements/spring/model/LinearSpringModel.hpp>

#include <Eigen/Geometry>
#include <math.h>

#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>
#include <mbslib/elements/spring/model/LinearSpringModel.hpp>
#include <mbslib/utility/DeriveOMat.hpp>

#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <math.h>

#define ENDPOINTFORCEID 0
#define ENDPOINTMEASUREID 1
#define ENDPOINTTCPID 2
#define ENDPOINTTOOLID 3

using namespace mbslib;

class RobotCreator {

public:
    RobotCreator() {
    }

    // Creates 2-DOF Roboter
    MbsCompoundWithBuilder * createRobot(std::vector< bool > & doDirDyn, TScalar l1 = 1, TScalar l2 = 1, TScalar m1 = 1, TScalar m2 = 1) {

        MbsCompoundWithBuilder * mbs = new MbsCompoundWithBuilder();
        mbs->addFixedBase("base");

        mbs->addFork();
        Joint1DOF * j0_drive = mbs->addRevoluteJoint(TVector3::UnitZ(), 0, "j0_drive");
        mbs->addEndpoint();
        Joint1DOF * j0_output = mbs->addRevoluteJoint(TVector3::UnitZ(), 0, "j0_output");
        LinearSpringModel sm0(555, 1);
        mbs->addSpring(sm0, *j0_drive, *j0_output, "spring0");
        doDirDyn.push_back(false); // InvDyn for drive
        doDirDyn.push_back(true);  // DirDyn for output

        mbs->addRigidLink(TVector3(l1, 0, 0), TVector3(l1 * 0.5, 0, 0), m1, Eigen::Matrix3d::Zero().cast< TScalar >(), "link_1");

        mbs->addFork();
        Joint1DOF * j1_drive = mbs->addRevoluteJoint(TVector3::UnitZ(), 0, "j1_drive");
        mbs->addEndpoint();
        Joint1DOF * j1_output = mbs->addRevoluteJoint(TVector3::UnitZ(), 0, "j1_output");
        LinearSpringModel sm1(666, 1);
        mbs->addSpring(sm1, *j1_drive, *j1_output, "spring1");
        doDirDyn.push_back(false); // InvDyn for drive
        doDirDyn.push_back(true);  // DirDyn for output

        mbs->addRigidLink(TVector3(l2, 0, 0), TVector3(l2 * 0.5, 0, 0), m2, Eigen::Matrix3d::Zero().cast< TScalar >(), "link_2");

        mbs->addEndpoint("tcp");

        mbs->setGravitation(TVector3(0, -9.81, 0));
        return mbs;
    };
};

mbslib::DeriveOMat * dom;
int n_indep; // Number of independent variables
int n_dep;   // Number of dependent variables

// ADOL-C Tape erstellen
void setupTape() {
    std::vector< bool > doDirDyn;
    RobotCreator robotCreator = RobotCreator();
    MbsCompound * mbs = robotCreator.createRobot(doDirDyn);

    dom = new DeriveOMat(*mbs);

    dom->addIndependent("j0_output", "q");
    dom->addIndependent("j1_output", "q");
    dom->addIndependent("j0_output", "dq");
    dom->addIndependent("j1_output", "dq");
    dom->addIndependent("spring0", "springconstant");
    dom->addIndependent("spring1", "springconstant");

    dom->addDependent("j0_output", "dq");
    dom->addDependent("j1_output", "dq");
    dom->addDependent("j0_output", "ddq");
    dom->addDependent("j1_output", "ddq");

    n_indep = dom->getIndependents();
    n_dep = dom->getDependents();

    double indep[n_indep];
    double dep[n_dep];

    for (int i = 0; i < n_indep; i++) {
        indep[i] = 1;
    }

    TVectorX q_soll(2);
    TVectorX dq_soll(2);
    TVectorX q_ist(2);
    TVectorX dq_ist(2);
    q_soll << -M_PI_2, 0.0;
    dq_soll << 0, 0;
    q_ist << -M_PI_2 + 0.1, 0;
    dq_ist << 0, 0;

    TVectorX q(4);
    TVectorX dq(4);
    q << q_soll(0), q_ist(0), q_soll(1), q_ist(1);
    dq << dq_soll(0), dq_ist(0), dq_soll(1), dq_ist(1);

    mbs->setJointPosition(q);
    mbs->setJointVelocity(dq);

    dom->startTape(indep);
    mbs->doABAhyb(doDirDyn);
    dom->endTape(dep);

    delete mbs;
}

int main(void) {
    using namespace mbslib;

    MbsCompoundWithBuilder mbs;

    mbs.addFixedBase();
    Joint1DOF & j1 = *mbs.addRevoluteJoint(Eigen::Vector3d::UnitZ().cast< TScalar >());
    mbs.addRigidLink(Eigen::Vector3d::UnitX().cast< TScalar >(), Eigen::Vector3d::Zero().cast< TScalar >(), 1, Eigen::Matrix3d::Zero().cast< TScalar >());
    //mbs.addFork();
    //mbs.addRigidLink(Eigen::Vector3d::Zero().cast<TScalar>(), Eigen::Vector3d::Zero().cast<TScalar>(), 1, Eigen::Matrix3d::Zero().cast<TScalar>());
    //mbs.addEndpoint();
    Joint1DOF & j2 = *mbs.addRevoluteJoint(Eigen::Vector3d::UnitZ().cast< TScalar >());
    mbs.addRigidLink(Eigen::Vector3d::UnitX().cast< TScalar >(), Eigen::Vector3d::Zero().cast< TScalar >(), 1, Eigen::Matrix3d::Zero().cast< TScalar >());
    Endpoint * tcp = mbs.addEndpoint();

    mbs.setGravitation(Eigen::Vector3d::UnitY().cast< TScalar >() * -1);

    j1.getJointState().q = 0;
    j1.getJointState().dotq = 1;
    j1.getJointState().ddotq = 0;
    j2.getJointState().q = 0;
    j2.getJointState().dotq = 0;
    j2.getJointState().ddotq = 0;

    mbs.doDirkin();
    mbs.doVelocity();
    std::cout << tcp->getCoordinateFrame().r << std::endl;
    std::cout << tcp->getCoordinateFrame().v << std::endl;

    double q1 = 0;
    double q2 = 0;
    double x, y, z;

    short int tag = 0;

    trace_on(tag);
    j1.getJointState().q <<= q1;
    j2.getJointState().q <<= q2;

    mbs.doDirkin();

    TVector3 rv = tcp->getCoordinateFrame().r;
    rv.x() >>= x;
    rv.y() >>= y;
    rv.z() >>= z;
    trace_off();

    double q[2];
    double r[3];
    double ** J = (double **)malloc(3 * sizeof(double *));
    double J0[2]; // = (double*)malloc(2*sizeof(double));
    double J1[2]; // = (double*)malloc(3*sizeof(double));
    double J2[2]; // = (double*)malloc(3*sizeof(double));
    J[0] = J0;
    J[1] = J1;
    J[2] = J2;

    q[0] = 0;
    q[1] = 0;
    ::function(0, 3, 2, q, r);
    ::jacobian(0, 3, 2, q, J);

    q[0] = 1.57;
    q[1] = 0;
    ::function(0, 3, 2, q, r);
    ::jacobian(0, 3, 2, q, J);

    return 0;
    mbs.doRne();

    const CoordinateFrame & cof = tcp->getCoordinateFrame();

    std::cout << mbs.getJointForceTorque() << std::endl;

    TVector3 extF;
    extF.setZero();
    TVector3 extN;
    extN.setZero();

    extF(0) = 2;

    tcp->setExternalForceTorque(extF, extN);

    mbs.doRne();
    std::cout << mbs.getJointForceTorque() << std::endl;

    mbs.doAggregateBody();
    const AggregateBody & ab = mbs.getBase().getAggregateBody();
    std::cout << std::endl
              << ab.m << std::endl
              << ab.com << std::endl
              << ab.I << std::endl;

    j2.setJointPosition(-3.141592 / 4);
    mbs.doDirkin();
    std::cout << std::endl
              << cof.r << std::endl;
    mbs.doAggregateBody();
    std::cout << std::endl
              << ab.m << std::endl
              << ab.com << std::endl
              << ab.I << std::endl;

    return 0;
}
