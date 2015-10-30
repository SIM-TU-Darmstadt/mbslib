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
 * \file src/example/exp_07_aba.cpp
 * 
 */
#include <mbslib/mbslib.hpp>

#include <Eigen/Geometry>

#include <iostream>

using namespace mbslib;

void test_this_later() {
    MbsCompoundWithBuilder c;
    Base * b = c.addFixedBase();
    c.addRevoluteJoint(TVector3(0, 0, 1));
    MbsObject * j2 = c.addPrismaticJoint(TVector3(1, 0, 0));
    c.addPrismaticJoint(TVector3(0, 1, 0));
    c.addRigidLink(TVector3(0, 0, 0), TVector3(0, 0, 0), 1, 1 * TMatrix3x3::Identity());
    Endpoint * e = c.addEndpoint();

    c.setJointPosition(TVector3(0, 0, 0));
    c.setJointVelocity(TVector3(1, 1, 0));
    c.setJointForceTorque(TVector3(0, 0, 0));

    //here we have different results for aba and crba. aba looks strange...
    //c.doDirkin();
    c.doABA();
    //c.doCrba();

    std::cout << "tau = " << c.getJointForceTorque().transpose() << std::endl;
    std::cout << "ddq = " << c.getJointAcceleration().transpose() << std::endl;
    std::cout << " dq = " << c.getJointVelocity().transpose() << std::endl;
    std::cout << "  q = " << c.getJointPosition().transpose() << std::endl;
    std::cout << "sacc = " << e->getCoordinateFrame().spatialAcceleration.transpose() << std::endl;
    std::cout << "svel = " << e->getCoordinateFrame().spatialVelocity.transpose() << std::endl;
    std::cout << "dom = " << e->getCoordinateFrame().dotomega << " dv = " << e->getCoordinateFrame().dotv << std::endl;

    c.doRne();
    std::cout << std::endl
              << c.getJointForceTorque().transpose() << std::endl;
}

void t1() {
    MbsCompoundWithBuilder c;
    Base * b = c.addFixedBase();
    c.addRevoluteJoint(TVector3(0, 0, 1));
    c.addPrismaticJoint(TVector3(0, 1, 0));
    MbsObject * j2 = c.addPrismaticJoint(TVector3(1, 0, 0));
    c.addRigidLink(TVector3(0, 0, 0), TVector3(0, 0, 0), 1, 1 * TMatrix3x3::Identity());
    Endpoint * e = c.addEndpoint();

    c.setJointPosition(TVector3(0, 0, 0));
    c.setJointVelocity(TVector3(1, 0, 1));
    c.setJointForceTorque(TVector3(0, 0, 0));

    //c.doDirkin();
    c.doABA();
    //c.doCrba();

    std::cout << "tau = " << c.getJointForceTorque().transpose() << std::endl;
    std::cout << "ddq = " << c.getJointAcceleration().transpose() << std::endl;
    std::cout << " dq = " << c.getJointVelocity().transpose() << std::endl;
    std::cout << "  q = " << c.getJointPosition().transpose() << std::endl;
    std::cout << "a2  = " << j2->getCoordinateFrame().spatialAcceleration.transpose() << std::endl;
    std::cout << "acc = " << e->getCoordinateFrame().spatialAcceleration.transpose() << std::endl;
    std::cout << "v2  = " << j2->getCoordinateFrame().spatialVelocity.transpose() << std::endl;
    std::cout << "vel = " << e->getCoordinateFrame().spatialVelocity.transpose() << std::endl;
    std::cout << "dom = " << e->getCoordinateFrame().dotomega << " dv = " << e->getCoordinateFrame().dotv << std::endl;

    c.doRne();
    std::cout << std::endl
              << c.getJointForceTorque().transpose() << std::endl;
}

int main() {
    test_this_later();
    t1();
    return 0;

    MbsCompoundWithBuilder c;

    Base * b = c.addFreeBase();
    c.addFork();
    c.addRigidLink(TVector3(1, 0, 0), TVector3(0, 0, 0), 1, 1 * TMatrix3x3::Identity());
    c.addEndpoint();
    Joint1DOF * j1; // = c.addPrismaticJoint(TVector3(1,0,0));
    //FixedRotation * r1 = c.addFixedRotation(TMatrix3x3(Eigen::AngleAxis<TScalar>(TScalar( 1 * M_PI/2 ) , TVector3(0,0,1) ) ) );
    //RigidLink * l1 =c.addRigidLink(TVector3(1,0,0),TVector3(0,0,0),1,0 * TMatrix3x3::Identity());
    Joint1DOF * j2 = c.addRevoluteJoint(TVector3(1, 0, 0));
    RigidLink * l2 = c.addRigidLink(TVector3(1, 0, 0), TVector3(0, 0, 0), 0, 0 * TMatrix3x3::Identity());
    Endpoint * e = c.addEndpoint();

    c.setGravitation(TVector3(00, 0, 0));

    c.doDirkin();

    //std::cout << c.calculateMassMatrix2() << std::endl << std::endl;

    j2->setJointForceTorque(0);

    e->setExternalForceTorque(TVector3(0, 0, 1), TVector3(0, 0, 0));

    c.doCrba();

    std::cout << b->getCoordinateFrame().dotv.transpose() << " ";
    std::cout << b->getCoordinateFrame().dotomega.transpose() << std::endl;
    std::cout << j2->getJointState().ddotq << std::endl;
    std::cout << e->getCoordinateFrame().dotv.transpose() << " ";
    std::cout << e->getCoordinateFrame().dotomega.transpose() << std::endl
              << std::endl;

    return 0;

    j1->getJointState().dotq = 1;
    j2->getJointState().dotq = 0;

    c.doRne();

    std::cout << j1->getJointForceTorque() << " " << j2->getJointForceTorque() << std::endl
              << std::endl;

    //return 0;

    j1->getJointState().dotq = 1;
    j2->getJointState().dotq = 0;
    j1->setJointForceTorque(0);
    j2->setJointForceTorque(-1);

    //  c.doABA();
    c.doCrba();

    //std::cout << e->getCoordinateFrame().r << std::endl << std::endl;
    //std::cout << e->getCoordinateFrame().dotv << std::endl << std::endl;
    //std::cout << b->getArticulatedBodyInertia() << std::endl << std::endl;
    //std::cout << l1->getArticulatedBodyInertia() << std::endl << std::endl;
    //std::cout << l2->getArticulatedBodyInertia() << std::endl << std::endl;
    std::cout << j1->getJointState().ddotq << " ";
    std::cout << j2->getJointState().ddotq << std::endl;
    std::cout << e->getCoordinateFrame().dotv << std::endl
              << std::endl;

    j1->getJointState().dotq = 2;
    j2->getJointState().dotq = 0;
    j1->setJointForceTorque(0);
    j2->setJointForceTorque(-4);

    //  c.doABA();
    c.doCrba();
    std::cout << std::endl;
    std::cout << j1->getJointState().ddotq << " ";
    std::cout << j2->getJointState().ddotq << std::endl; // << std::endl;
    std::cout << e->getCoordinateFrame().dotv << std::endl
              << std::endl;
    //std::cout << j2->getJointState().ddotq << std::endl;
}