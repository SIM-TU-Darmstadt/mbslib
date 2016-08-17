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
 * \file mbslib/elements/fork/Fork.cpp
 * Definition of mbslib::Fork
 */

#include <Eigen/Geometry>
#include <mbslib/elements/fork/Fork.hpp>
#include <mbslib/utility/internalTests.hpp>

using namespace mbslib;

Fork::Fork(MbsObject & pred, const std::string & name)
    : MbsObjectTwoSucc(pred, name.length() == 0 ? "unnamedFork" : name) {
}

//void Fork::doPosition()
//{
//  cof.r = pred->getCoordinateFrame().r;
//  cof.R = pred->getCoordinateFrame().R;
//}

void Fork::doVelocity() {
    cof.omega = pred->getCoordinateFrame().omega;
    cof.v = pred->getCoordinateFrame().v;
}

void Fork::doAcceleration() {
    cof.dotomega = pred->getCoordinateFrame().dotomega;
    cof.dotv = pred->getCoordinateFrame().dotv;
}

void Fork::doRneInward(bool withExternalForce) {
    rneForceTorque.f = succOne->getRneForceTorque().f + succTwo->getRneForceTorque().f;
    rneForceTorque.n = succOne->getRneForceTorque().n + succTwo->getRneForceTorque().n;
}

void Fork::doAggregateBody() {
    TScalar m1 = succOne->getAggregateBody().m;
    const TVector3 & com1 = succOne->getAggregateBody().com;
    const TMatrix3x3 & I1 = succOne->getAggregateBody().I;
    checkInertia(m1, I1);

    TScalar m2 = succTwo->getAggregateBody().m;
    const TVector3 & com2 = succTwo->getAggregateBody().com;
    const TMatrix3x3 & I2 = succTwo->getAggregateBody().I;
    checkInertia(m2, I2);

    TVector3 ccom;
    if ((m1 + m2) != 0) {
        ccom = (m1 * com1 + m2 * com2) / (m1 + m2);
    } else {
        ccom = TVector3::Zero();
    }
    TVector3 dcom1 = com1 - ccom;
    TVector3 dcom2 = com2 - ccom;

    aggregateBody.m = m1 + m2;
    aggregateBody.com = ccom;
    aggregateBody.I = I1 + m1 * (dcom1.dot(dcom1) * TMatrix3x3::Identity() - dcom1 * dcom1.transpose()) + I2 + m2 * (dcom2.dot(dcom2) * TMatrix3x3::Identity() - dcom2 * dcom2.transpose());

    checkInertia(aggregateBody.m, aggregateBody.I);
}

void Fork::doABASweep1fwd() {
    cof.spatialVelocity = pred->getCoordinateFrame().spatialVelocity;
}

void Fork::doABASweep2fwd() {
    // simply sum up ABI and bias-force from the successors
    articulatedBodyInertia = succOne->getArticulatedBodyInertia() + succTwo->getArticulatedBodyInertia();
    biasForce = succOne->getBiasForce() + succTwo->getBiasForce();
}

void Fork::doABASweep3fwd() {
    cof.spatialAcceleration = pred->getCoordinateFrame().spatialAcceleration;
}

int Fork::getDOF() const {
    return 0;
}

bool Fork::isConnected() const {
    return ((succOne != NULL) && (succTwo != NULL));
}

void Fork::doABASweep1inv() {
    doABASweep1fwd();
}

void Fork::doABASweep2inv() {
    doABASweep2fwd();
}

void Fork::doABASweep3inv() {
    doABASweep3fwd();
}

void Fork::makeJacobian(TMatrix6xX & /*jacobian*/, const TVector3 & /*referencePoint*/) const {
    return;
}

Fork::~Fork() {
}
