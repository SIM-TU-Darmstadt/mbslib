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
 * \file mbslib/elements/endpoint/EndpointMassless.cpp
 * Definition of mbslib::EndpointMassless
 */

#include <mbslib/elements/endpoint/EndpointMassless.hpp>
#include <mbslib/utility/mathtools.hpp>
#include <Eigen/Geometry>

using namespace mbslib;

EndpointMassless::EndpointMassless(MbsObject & pred, const std::string & name)
    : Endpoint(pred, name) {
    extForce.setZero();
    extTorque.setZero();
    aggregateBody.m = 0;
    aggregateBody.com.setZero();
    aggregateBody.I.setZero();
    articulatedBodyInertia.setZero();
}

//void EndpointMassless::doPosition()
//{
//  cof.r = pred->getCoordinateFrame().r;
//  cof.R = pred->getCoordinateFrame().R;
//}

void EndpointMassless::doVelocity() {
    cof.omega = pred->getCoordinateFrame().omega;
    cof.v = pred->getCoordinateFrame().v;
}

void EndpointMassless::doAcceleration() {
    cof.dotomega = pred->getCoordinateFrame().dotomega;
    cof.dotv = pred->getCoordinateFrame().dotv;
}

void EndpointMassless::doRneInward(bool withExternalForce) {
    if (!withExternalForce) {
        rneForceTorque.f.setZero();
        rneForceTorque.n.setZero();
    } else {
        rneForceTorque.f = -extForce;
        rneForceTorque.n = -extTorque;
    }
    //std::cout << "EndpointMassless::doRneInward " << withExternalForce << " " << name << " dotv=" << cof.dotv.transpose() << " m=" << m << std::endl;
}

void EndpointMassless::doAggregateBody() {
}

void EndpointMassless::doABASweep1fwd() {
    cof.spatialVelocity = pred->getCoordinateFrame().spatialVelocity;
    // nothing to do
    // and isolated bias force is calculated in sweep2
}

void EndpointMassless::doABASweep2fwd() {
    // we need not calculated the ABI, as it is set in the constructor and independent of any successors (as there are none)
    // bias force needs to consider external forces:
    biasForce.segment< 3 >(0) = -extTorque;
    biasForce.segment< 3 >(3) = -extForce;
    // and now we add the bias force required because of motion of the link (this can be simplified later on!)
    // note that the articulated body inertia of this element is the same as the local inertia
    TVector6 p = spatialCrossStar(cof.spatialVelocity, articulatedBodyInertia * cof.spatialVelocity);
    biasForce = biasForce + p;
}

void EndpointMassless::doABASweep3fwd() {
    cof.spatialAcceleration = pred->getCoordinateFrame().spatialAcceleration;
    // nothing to do, as we have neither a joint nor a free base here
}

void EndpointMassless::addExternalForceTorque(const TVector3 & f, const TVector3 & t) {
    extForce += f;
    extTorque += t;
}

void EndpointMassless::addExternalForceTorqueWCS(const TVector3 & f, const TVector3 & t) {
    extForce += cof.R.transpose() * f;
    extTorque += cof.R.transpose() * t;
}
void EndpointMassless::setExternalForceTorque(const TVector3 & f, const TVector3 & t) {
    extForce = f;
    extTorque = t;
}

void EndpointMassless::setExternalForceTorqueWCS(const TVector3 & f, const TVector3 & t) {
    extForce = cof.R.transpose() * f;
    extTorque = cof.R.transpose() * t;
}

/**
 * \brief First (outward) sweep of inverse dynamics ABA.
 */
void EndpointMassless::doABASweep1inv() {
    doABASweep1fwd();
}

/**
 * \brief Second (inward) sweep of inverse dynamics ABA.
 */
void EndpointMassless::doABASweep2inv() {
    doABASweep2fwd();
}

/**
 * \brief Third (outward) sweep of inverse dynamics ABA.
 */
void EndpointMassless::doABASweep3inv() {
    doABASweep3fwd();
}

void EndpointMassless::makeJacobian(TMatrix6xX & /*jacobian*/, const TVector3 & /*referencePoint*/) const {
    return;
}

const TVector3 & EndpointMassless::getExternalForce() const {
    return extForce;
}

const TVector3 & EndpointMassless::getExternalTorque() const {
    return extTorque;
}

EndpointMassless::~EndpointMassless() {
}
