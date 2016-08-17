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
#include <mbslib/mbslib.hpp>

#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

void test1() {
    using namespace mbslib;
    TVector3 a(1, 2, 3), a2(3, 2, 1);
    std::cout << a.transpose() << std::endl;
    TVectorX b(3);
    b = a;
    std::cout << b.transpose() << std::endl;
    std::cout << a.cross(a2).transpose() << std::endl;

    TMatrix3x3 A;
    A.setIdentity();
    std::cout << (A * a + a2).transpose() << std::endl;
}

int main(void) {
    test1();
    using namespace mbslib;

    MbsCompoundWithBuilder mbs;

    mbs.addFixedBase();
    Joint1DOF & j1 = *mbs.addRevoluteJoint(TVector3::UnitZ());
    //std::cout << "j1 axis: " << ((RevoluteJoint&)j1).getAxis().transpose() << std::endl;
    mbs.addRigidLink(TVector3::UnitX(), TVector3::Zero(), 1, TMatrix3x3::Zero());
    //mbs.addFork();
    //mbs.addRigidLink(Eigen::Vector3d::Zero().cast<TScalar>(), Eigen::Vector3d::Zero().cast<TScalar>(), 1, Eigen::Matrix3d::Zero().cast<TScalar>());
    //mbs.addEndpoint();
    Joint1DOF & j2 = *mbs.addRevoluteJoint(TVector3::UnitZ());
    //std::cout << "j2 axis: " << ((RevoluteJoint&)j2).getAxis().transpose() << std::endl;
    mbs.addRigidLink(TVector3::UnitX(), TVector3::Zero(), 1, TMatrix3x3::Zero());
    Endpoint * tcp = mbs.addEndpoint();

    mbs.setGravitation(TVector3::UnitY() * -1);

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
