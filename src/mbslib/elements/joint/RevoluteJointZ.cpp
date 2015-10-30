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
 * \file mbslib/elements/joint/RevoluteJointZ.cpp
 * Definition of mbslib::RevoluteJointZ
 */

#include <mbslib/elements/joint/RevoluteJointZ.hpp>
#include <Eigen/Geometry>

using namespace mbslib;

RevoluteJointZ::RevoluteJointZ(MbsObject & pred, int jointId, int stateVectorPosition, TScalar jointOffset, TScalar jointFriction, TScalar gearRatio, TScalar rotorInertia, const std::string & name)
    : RevoluteJoint(pred, TVector3(0, 0, 1), jointId, stateVectorPosition, jointOffset, jointFriction, gearRatio, rotorInertia, name) {
    this->axis = axis;
    if (axis.norm() == 0) {
        setupError = true;
    } else {
        this->axis.normalize();
    }
    //relR.setIdentity();
}

void RevoluteJointZ::doVelocity() {
    cof.omega = getRelativeOrientation().transpose() * pred->getCoordinateFrame().omega + axis * jp.dotq;
    cof.v = getRelativeOrientation().transpose() * pred->getCoordinateFrame().v;
}

void RevoluteJointZ::doAcceleration() {
    cof.dotomega = getRelativeOrientation().transpose() * pred->getCoordinateFrame().dotomega + axis * jp.ddotq + cof.omega.cross(axis * jp.dotq);
    cof.dotv = getRelativeOrientation().transpose() * pred->getCoordinateFrame().dotv;
}

void RevoluteJointZ::doRneInward(bool withExternalForce) {
    rneForceTorque.f = getRelativeOrientation() * succ->getRneForceTorque().f;
    rneForceTorque.n = getRelativeOrientation() * succ->getRneForceTorque().n;
    tau = axis.dot(rneForceTorque.n) + jp.dotq * jointFriction + squaredGearRotorInertia * jp.ddotq - externalTau;
}

void RevoluteJointZ::doAggregateBody() {
    aggregateBody.m = succ->getAggregateBody().m;
    aggregateBody.I = getRelativeOrientation() * succ->getAggregateBody().I * getRelativeOrientation().transpose();
    aggregateBody.com = getRelativeOrientation() * succ->getAggregateBody().com;
}

TScalar RevoluteJointZ::calculateCRBAforce(TVector3 & f, TVector3 & n) {
    const AggregateBody & ab = succ->getAggregateBody();
    //TVector3 ax = axis;
    //f = ax.cross ( ab.com ) * ab.m;
    f = TVector3(-ab.m * ab.com(1), ab.m * ab.com(0), 0);
    //n = ab.com.cross(f) + ab.I * ax;
    n = ab.com.cross(f) + ab.I.col(2);
    //TScalar tau = n.dot( ax );
    TScalar tau = n(2);
    f = cof.R * f;
    n = cof.R * n;
    return tau + squaredGearRotorInertia;
}

TScalar RevoluteJointZ::calculateCRBAforce(const TVector3 & pos, const TVector3 & f, const TVector3 & n) {
    //The following two lines is the general version
    //(1) calculate resulting local torque
    //TVector3 nlocal =  cof.R.transpose() * ( n + ( pos - cof.r ).cross(f));
    //(2) calculate inner product.
    //return nlocal.dot( axis );
    //As we know, that the axis is alwais (0 0 1), we can use the last element instead.
    //return nlocal[2];

    // even better: the inner procuct can be applied from the left (as a row matrix), thus selecting
    // the 3rd row of R^T (being the 3rd collum of R)
    return cof.R.col(2).dot(n + (pos - cof.r).cross(f));
}

void RevoluteJointZ::makeJacobian(TMatrix6xX & jacobian, const TVector3 & referencePoint) const {
    jacobian.block< 3, 1 >(0, stateVectorPosition) = cof.R.col(2);
    jacobian.block< 3, 1 >(3, stateVectorPosition) = (cof.R.col(2)).cross(referencePoint - cof.r);
}

void RevoluteJointZ::calcRelPose(TVector3 & relr, TMatrix3x3 & relR) {
    //const CoordinateFrame & predCof = pred->getCoordinateFrame();
    //cof.r = predCof.r;
    relR = Eigen::AngleAxis< TScalar >((jp.q + jointOffset), axis);
    //cof.R = predCof.R * relR;
}

RevoluteJointZ::~RevoluteJointZ() {
}
