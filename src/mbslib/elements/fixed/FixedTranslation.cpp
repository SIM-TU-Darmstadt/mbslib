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
 * \file mbslib/endpoint/fixed/FixedTranslation.cpp
 * Definition of mbslib::FixedTranslation
 */

#include <mbslib/elements/fixed/FixedTranslation.hpp>
#include <mbslib/utility/mathtools.hpp>
#include <Eigen/Geometry>

using namespace mbslib;

FixedTranslation::FixedTranslation(MbsObject & pred, const TVector3 & relr, const std::string & name)
    : MbsObjectOneSucc(pred, name) {
    this->fixedRelr = relr;
}

void FixedTranslation::doVelocity() {
    cof.omega = pred->getCoordinateFrame().omega;
    cof.v = pred->getCoordinateFrame().v + cof.omega.cross(this->getRelativePosition());
}

void FixedTranslation::doAcceleration() {
    cof.dotomega = pred->getCoordinateFrame().dotomega;
    cof.dotv = pred->getCoordinateFrame().dotv + cof.dotomega.cross(this->getRelativePosition()) + cof.omega.cross(cof.omega.cross(this->getRelativePosition()));
}

void FixedTranslation::doRneInward(bool withExternalForce) {
    rneForceTorque.f = succ->getRneForceTorque().f;
    rneForceTorque.n = succ->getRneForceTorque().n + getRelativePosition().cross(succ->getRneForceTorque().f);
}

void FixedTranslation::doAggregateBody() {
    aggregateBody.m = succ->getAggregateBody().m;
    aggregateBody.com = getRelativePosition() + succ->getAggregateBody().com;
    aggregateBody.I = succ->getAggregateBody().I;
    return;
}

void FixedTranslation::doABASweep1fwd() {
    cof.spatialVelocity = makeVelocityTransform(TVector3(-getRelativePosition())) * pred->getCoordinateFrame().spatialVelocity;
    // no bias force, as there is no mass/inertia
}

void FixedTranslation::doABASweep2fwd() {
    // Translate ABI and bias force along the link
    // no further calculations, as this link is massless

    /*
  articulatedBodyInertia = makeForceTransform( TVector3( getRelativePosition() ) ) * succ->getArticulatedBodyInertia() * makeVelocityTransform( TVector3( -getRelativePosition() ) );
  biasForce = makeForceTransform(TVector3(getRelativePosition())) * succ->getBiasForce();
  */
    transformABI(articulatedBodyInertia, getRelativePosition(), succ->getArticulatedBodyInertia());
    transformForce(biasForce, getRelativePosition(), succ->getBiasForce());

    //std::cout << "fixt - biasForce = " << biasForce.transpose() << std::endl;
    /*
  std::cout << "rl -   p = " << p.transpose() << std::endl;
  std::cout << "rl - vel = " << cof.spatialVelocity.transpose() << std::endl;
  std::cout << "rl - acc = " << cof.spatialAcceleration.transpose() << std::endl;
  std::cout << "rl - biasForce = " << biasForce.transpose() << std::endl;
  std::cout << "rl - abaI = " << std::endl << abaI <<std::endl;
  std::cout << "rl - abi = " << std::endl << articulatedBodyInertia << std::endl;
  */
}

void FixedTranslation::doABASweep3fwd() {
    // Just propagate the acceleration along the link
    TVector6 atmp = makeVelocityTransform(TVector3(-1 * getRelativePosition())) * pred->getCoordinateFrame().spatialAcceleration;
    //TVector6 atmp = makeVelocityTransform((-1 * getRelativePosition())) * pred->getCoordinateFrame().spatialAcceleration;
    cof.spatialAcceleration = atmp;
}

int FixedTranslation::getDOF() const {
    return 0;
}

const TVector3 & FixedTranslation::getFixedRelr() const {
    return fixedRelr;
}

void FixedTranslation::doABASweep1inv() {
    doABASweep1fwd();
}

void FixedTranslation::doABASweep2inv() {
    doABASweep2fwd();
}

void FixedTranslation::doABASweep3inv() {
    doABASweep3fwd();
}

void FixedTranslation::makeJacobian(TMatrix6xX & /*jacobian*/, const TVector3 & /*referencePoint*/) const {
    return;
}

void FixedTranslation::calcRelPose(TVector3 & relr, TMatrix3x3 & relR) {
    relr = fixedRelr;
}

FixedTranslation::~FixedTranslation() {
}
