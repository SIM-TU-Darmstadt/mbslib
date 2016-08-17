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
 * \file mbslib/elements/endpoint/Endpoint.cpp
 * Definition of mbslib::Endpoint
 */

#include <Eigen/Geometry>
#include <mbslib/elements/endpoint/Endpoint.hpp>
#include <mbslib/utility/mathtools.hpp>

using namespace mbslib;

Endpoint::Endpoint(MbsObject & pred, const std::string & name)
    : MbsObjectNoSucc(pred, name)
    , m(0)
    , I(TMatrix3x3::Zero()) {
    extForce.setZero();
    extTorque.setZero();
    aggregateBody.m = 0;
    aggregateBody.com.setZero();
    aggregateBody.I.setZero();
    articulatedBodyInertia.setZero();
}

Endpoint::Endpoint(MbsObject & pred, TScalar m, const TMatrix3x3 & I, const std::string & name)
    : MbsObjectNoSucc(pred, name)
    , m(m)
    , I(I) {
    extForce.setZero();
    extTorque.setZero();
    aggregateBody.m = m;
    aggregateBody.com.setZero();
    aggregateBody.I = I;
    ///TODO: in a parameter-optimization scenario, this calculation must be done in each step, as parameters may change
    articulatedBodyInertia.setZero();
    articulatedBodyInertia.block< 3, 3 >(0, 0) = I;
    articulatedBodyInertia.block< 3, 3 >(3, 3) = TMatrix3x3::Identity() * m;
}

//void Endpoint::doPosition()
//{
//  cof.r = pred->getCoordinateFrame().r;
//  cof.R = pred->getCoordinateFrame().R;
//}

void Endpoint::doVelocity() {
    cof.omega = pred->getCoordinateFrame().omega;
    cof.v = pred->getCoordinateFrame().v;
}

void Endpoint::doAcceleration() {
    cof.dotomega = pred->getCoordinateFrame().dotomega;
    cof.dotv = pred->getCoordinateFrame().dotv;
}

void Endpoint::doRneInward(bool withExternalForce) {
    if (!withExternalForce) {
        rneForceTorque.f.setZero();
        rneForceTorque.n.setZero();
    } else {
        rneForceTorque.f = -extForce;
        rneForceTorque.n = -extTorque;
    }
    rneForceTorque.f += cof.dotv * m;
    rneForceTorque.n += (I * cof.dotomega + cof.omega.cross(I * cof.omega));
    //std::cout << "Endpoint::doRneInward " << withExternalForce << " " << name << " dotv=" << cof.dotv.transpose() << " m=" << m << std::endl;
}

void Endpoint::doAggregateBody() {
    aggregateBody.m = m;
    aggregateBody.I = I;
}

void Endpoint::doABASweep1fwd() {
    cof.spatialVelocity = pred->getCoordinateFrame().spatialVelocity;
    // nothing further to do here
    // bias force is calculated in sweep2
}

void Endpoint::doABASweep2fwd() {
    articulatedBodyInertia.setZero();
    articulatedBodyInertia.block< 3, 3 >(0, 0) = I;
    articulatedBodyInertia.block< 3, 3 >(3, 3) = TMatrix3x3::Identity() * m;

    // we need not calculated the ABI, as it is set in the constructor and independent of any successors (as there are none)
    // bias force needs to consider external forces:
    biasForce.segment< 3 >(0) = -extTorque;
    biasForce.segment< 3 >(3) = -extForce;
    // and now we add the bias force required because of motion of the link (this can be simplified later on!)
    // note that the articulated body inertia of this element is the same as the local inertia
    TVector6 p = spatialCrossStar(cof.spatialVelocity, articulatedBodyInertia * cof.spatialVelocity);
    biasForce = biasForce + p;

    //std::cout << "ep -- ABI " << std::endl << articulatedBodyInertia << std::endl;
    //std::cout << "ep -- spacVel    = " << cof.spatialVelocity.transpose() << std::endl;
    //std::cout << "ep -- biasForce  = " << biasForce.transpose() << std::endl;
}

void Endpoint::doABASweep3fwd() {
    cof.spatialAcceleration = pred->getCoordinateFrame().spatialAcceleration;
    // nothing to do, as we have neither a joint nor a free base here
}

void Endpoint::addExternalForceTorque(const TVector3 & f, const TVector3 & t) {
    extForce += f;
    extTorque += t;
}

void Endpoint::addExternalForceTorqueWCS(const TVector3 & f, const TVector3 & t) {
    extForce += cof.R.transpose() * f;
    extTorque += cof.R.transpose() * t;
}
void Endpoint::setExternalForceTorque(const TVector3 & f, const TVector3 & t) {
    extForce = f;
    extTorque = t;
}

void Endpoint::setExternalForceTorqueWCS(const TVector3 & f, const TVector3 & t) {
    extForce = cof.R.transpose() * f;
    extTorque = cof.R.transpose() * t;
}

/**
 * \brief First (outward) sweep of inverse dynamics ABA.
 */
void Endpoint::doABASweep1inv() {
    doABASweep1fwd();
}

/**
 * \brief Second (inward) sweep of inverse dynamics ABA.
 */
void Endpoint::doABASweep2inv() {
    doABASweep2fwd();
}

/**
 * \brief Third (outward) sweep of inverse dynamics ABA.
 */
void Endpoint::doABASweep3inv() {
    doABASweep3fwd();
}

void Endpoint::makeJacobian(TMatrix6xX & /*jacobian*/, const TVector3 & /*referencePoint*/) const {
    return;
}

const TVector3 & Endpoint::getExternalForce() const {
    return extForce;
}

const TVector3 & Endpoint::getExternalTorque() const {
    return extTorque;
}

const TScalar & Endpoint::getMass() const {
    return this->m;
}

const TMatrix3x3 & Endpoint::getInertiaTensor() const {
    return this->I;
}

Endpoint::~Endpoint() {
}
