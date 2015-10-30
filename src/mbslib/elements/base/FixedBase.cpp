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
 * \file mbslib/elements/base/FixedBase.cpp
 * Definition of mbslib::FixedBase
 */

#include <mbslib/elements/base/FixedBase.hpp>

using namespace mbslib;

FixedBase::FixedBase(const std::string & name)
    : Base(name) {
    cof.r.setZero();
    cof.R.setIdentity();
    cof.v.setZero();
    cof.omega.setZero();
    cof.dotv.setZero();
    cof.dotomega.setZero();
    cof.spatialAcceleration.setZero();
    cof.spatialVelocity.setZero();
}

FixedBase::FixedBase(const TVector3 & position, const TMatrix3x3 & orientation, const std::string & name)
    : Base(name) {
    cof.r = position;
    cof.R = orientation;
    cof.v.setZero();
    cof.omega.setZero();
    cof.dotv.setZero();
    cof.dotomega.setZero();
    cof.spatialAcceleration.setZero();
    cof.spatialVelocity.setZero();
}

FixedBase::~FixedBase() {
}

int FixedBase::getDOF() const {
    return 0;
}

void FixedBase::doPosition() {
}

void FixedBase::doVelocity() {
}

void FixedBase::doAcceleration() {
}

void FixedBase::doRneInward(bool withExternalForce) {
    rneForceTorque.f = succ->getRneForceTorque().f;
    rneForceTorque.n = succ->getRneForceTorque().n;
}

void FixedBase::doAggregateBody() {
    aggregateBody = succ->getAggregateBody();
}

void FixedBase::doABASweep1fwd() {
    cof.spatialVelocity.setZero();
    cof.spatialAcceleration.segment< 3 >(0).setZero();
    cof.spatialAcceleration.segment< 3 >(3) = cof.dotv;
}

void FixedBase::doABASweep2fwd() {
    articulatedBodyInertia = succ->getArticulatedBodyInertia();
    biasForce = succ->getBiasForce();
}

void FixedBase::doABASweep3fwd() {
    //nothing to do here
}

void FixedBase::doABASweep1inv() {
    doABASweep1fwd();
}
void FixedBase::doABASweep2inv() {
    doABASweep2fwd();
}

void FixedBase::doABASweep3inv() {
    cof.spatialAcceleration.segment< 3 >(0).setZero();
    cof.spatialAcceleration.segment< 3 >(3) = cof.dotv;
}

void FixedBase::makeJacobian(TMatrix6xX & /*jacobian*/, const TVector3 & /*referencePoint*/) const {
    return;
}

void FixedBase::integrate(TTime /*dt*/) {
}

void FixedBase::storeState() {
}

void FixedBase::restoreState() {
}

void FixedBase::calcRelPose(TVector3 & /*relr*/, TMatrix3x3 & /*relR*/) {
}
