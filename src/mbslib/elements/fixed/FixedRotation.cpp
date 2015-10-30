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
 * \file mbslib/endpoint/fixed/FixedRotation.cpp
 * Definition of mbslib::FixedRotation
 */

#include <mbslib/elements/fixed/FixedRotation.hpp>
#include <mbslib/utility/mathtools.hpp>

using namespace mbslib;

FixedRotation::FixedRotation(MbsObject & pred, const TMatrix3x3 & rr, const std::string & name)
    : MbsObjectOneSucc(pred, name) {
    fixedRelR = rr;
}

//void FixedRotation::doPosition()
//{
//  cof.R = pred->getCoordinateFrame().R * getRelativeOrientation();
//  cof.r = pred->getCoordinateFrame().r;
//}

void FixedRotation::doVelocity() {
    cof.v = getRelativeOrientation().transpose() * pred->getCoordinateFrame().v;
    cof.omega = getRelativeOrientation().transpose() * pred->getCoordinateFrame().omega;
}

void FixedRotation::doAcceleration() {
    cof.dotv = getRelativeOrientation().transpose() * pred->getCoordinateFrame().dotv;
    cof.dotomega = getRelativeOrientation().transpose() * pred->getCoordinateFrame().dotomega;
}

void FixedRotation::doRneInward(bool withExternalForce) {
    rneForceTorque.f = getRelativeOrientation() * succ->getRneForceTorque().f;
    rneForceTorque.n = getRelativeOrientation() * succ->getRneForceTorque().n;
}

void FixedRotation::doAggregateBody() {
    aggregateBody.m = succ->getAggregateBody().m;
    aggregateBody.com = getRelativeOrientation() * succ->getAggregateBody().com;
    aggregateBody.I = getRelativeOrientation() * succ->getAggregateBody().I * getRelativeOrientation().transpose();
}

void FixedRotation::doABASweep1fwd() {
    cof.spatialVelocity = makeVelocityTransform(TMatrix3x3(getRelativeOrientation().transpose())) * pred->getCoordinateFrame().spatialVelocity;
    // nothing further to do here, all relevant values are zero
}

void FixedRotation::doABASweep2fwd() {
    // nothing to calculate, we just rotate the ABI and bias force

    transformABI(articulatedBodyInertia, getRelativeOrientation(), succ->getArticulatedBodyInertia());
    transformForce(biasForce, getRelativeOrientation(), succ->getBiasForce());
}

void FixedRotation::doABASweep3fwd() {
    TMatrix3x3 Rinv = getRelativeOrientation().transpose();
    TVector6 atmp = makeVelocityTransform(Rinv) * pred->getCoordinateFrame().spatialAcceleration;
    cof.spatialAcceleration = atmp;
    // nothing to do here, as we have neither a joint nor a free base
}

int FixedRotation::getDOF() const {
    return 0;
}

const TMatrix3x3 & FixedRotation::getFixedRotation() const {
    return fixedRelR;
}

void FixedRotation::doABASweep1inv() {
    doABASweep1fwd();
}

void FixedRotation::doABASweep2inv() {
    doABASweep2fwd();
}

void FixedRotation::doABASweep3inv() {
    doABASweep3fwd();
}

void FixedRotation::makeJacobian(TMatrix6xX & /*jacobian*/, const TVector3 & /*referencePoint*/) const {
    return;
}

void FixedRotation::calcRelPose(TVector3 & /*relr*/, TMatrix3x3 & relR) {
    relR = fixedRelR;
}

FixedRotation::~FixedRotation() {
}
