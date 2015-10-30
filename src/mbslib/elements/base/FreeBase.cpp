/**
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
 * The MBSlib is distributed WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with MBSlib.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \file mbslib/elements/base/FreeBase.cpp
 * Definition of mbslib::FreeBase
 */

#include <mbslib/elements/base/FreeBase.hpp>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>

using namespace mbslib;

FreeBase::FreeBase(const std::string & name)
    : Base(name) {
    cof.r.setZero();
    cof.R.setIdentity();
    cof.v.setZero();
    cof.omega.setZero();
    cof.dotv.setZero();
    cof.dotomega.setZero();
}

FreeBase::FreeBase(const TVector3 & position, const TMatrix3x3 & orientation, const std::string & name)
    : Base(name) {
    cof.r = position;
    cof.R = orientation;
    cof.v.setZero();
    cof.omega.setZero();
    cof.dotv.setZero();
    cof.dotomega.setZero();
}

FreeBase::~FreeBase() {
}

int FreeBase::getDOF() const {
    return 6;
}

void FreeBase::doPosition() {
}

void FreeBase::doVelocity() {
}

void FreeBase::doAcceleration() {
}

void FreeBase::doRneInward(bool withExternalForce) {
    rneForceTorque.f = succ->getRneForceTorque().f;
    rneForceTorque.n = succ->getRneForceTorque().n;
}

void FreeBase::doAggregateBody() {
    aggregateBody = succ->getAggregateBody();
}

void FreeBase::doABASweep1fwd() {
    doPosition();
    cof.spatialVelocity.segment< 3 >(0) = cof.omega;
    cof.spatialVelocity.segment< 3 >(3) = cof.v;

    //calculate c, whic is required in further calculations
    //we can ommit cJ, as Scirc is zero for this element
    //v is only dependent on the local velocity, we have no predecessor --> v = vJ;
    TVector6 v;
    v.segment< 3 >(0) = cof.omega;
    v.segment< 3 >(3) = cof.v;
    // c is the spatial crossproduct of v x vJ being [omega; v] x [omrel ; vel] = [omega x omrel ; v x omrel + om x vrel]
    // the upper part is 0 as A x A = 0
    // the lower part is 0 as om = omrel and v = vrel and A x B = -B x A
    c.setZero();
    // calculation of p is postponed to sweep 2
}

void FreeBase::doABASweep1inv() {
    doPosition();
    cof.spatialVelocity.segment< 3 >(0) = cof.omega;
    cof.spatialVelocity.segment< 3 >(3) = cof.v;

    //calculate c, whic is required in further calculations
    //we can ommit cJ, as Scirc is zero for this element
    //v is only dependent on the local velocity, we have no predecessor --> v = vJ;
    TVector6 v;
    v.segment< 3 >(0) = cof.omega;
    v.segment< 3 >(3) = cof.v;
    // c is the spatial crossproduct of v x vJ being [omega; v] x [omrel ; vel] = [omega x omrel ; v x omrel + om x vrel]
    // the upper part is 0 as A x A = 0
    // the lower part is 0 as om = omrel and v = vrel and A x B = -B x A
    c.setZero();
    // calculation of p is postponed to sweep 2

    assert(0); /// TODO calculate ddq from dotv and dotomega and set c = S * ddq;
}

void FreeBase::doABASweep2fwd() {
    // no addition needed here, as the base has no Inertia
    TMatrix6x6 IA = succ->getArticulatedBodyInertia();
    // calculation of p is not required, as we have no external force and no Inertia here
    // thus calculation of pA simplifies to taking pA from the successor
    TVector6 pA = succ->getBiasForce();
    // U and D are simple
    U = IA; // as S = Identity;
    D = U;  // S = Identity, again :-)
    // u is quite simple, too as tau is zero and S is Identity
    u = -pA;
    // articulated body inertia at handle is zero
    // Ia = IA - U * inv(D) * U = IA - IA * inv(IA) * IA = IA - IA
    articulatedBodyInertia.setZero();
    // bias force: pa = pA + Ia * c + U*inv(D)*u --- we have U = D = IA
    // simplifies  pa = pA + 0 *  c + IA * inv(IA) * u --- we have u = -pA
    // simplifies  pa = pA +                     1 * -pA = 0
    biasForce.setZero();
    //std::cout << "freebase -- u = "  << u.transpose() << std::endl;
    //std::cout << "freebase -- D = " << std::endl << D << std::endl;
}

void FreeBase::doABASweep2inv() {
    // no addition needed here, as the base has no Inertia
    TMatrix6x6 IA = succ->getArticulatedBodyInertia();
    // calculation of p is not required, as we have no external force and no Inertia here
    // thus calculation of pA simplifies to taking pA from the successor
    TVector6 pA = succ->getBiasForce();

    // calculation ABI and biasForce
    articulatedBodyInertia = IA;
    biasForce = pA + articulatedBodyInertia * c;

    assert(0); /// TODO: this one only works if sweep one is implemented!
}

void FreeBase::doABASweep3fwd() {
    //solve for acceleration
    TVector6 a;

#if EIGEN_WORLD_VERSION >= 3
    a = D.lu().solve(u);
#else // asume eigen version 2
    D.lu().solve(u, &a);
#endif
    cof.spatialAcceleration = a;
    // we have to calculate the acceleration of the base from the spatial acceleration
    // Note: spatial acc is in the _moving_ frame, while dotom and dotv are in a none-moving frame

    cof.dotomega = a.segment< 3 >(0);
    cof.dotv = a.segment< 3 >(3) + cof.omega.cross(cof.v);
}

void FreeBase::doABASweep3inv() {
    cof.spatialAcceleration = c;
    // we also could calculate tau here, but the free base has no place to but it :-)
    //TVector6 tau = articulatedBodyInertia * cof.spatialAcceleration + biasForce;

    // calculate classical acceleration
    cof.dotomega = cof.spatialAcceleration.segment< 3 >(0);
    cof.dotv = cof.spatialAcceleration.segment< 3 >(3) + cof.omega.cross(cof.v);
}

void FreeBase::makeJacobian(TMatrix6xX & jacobian, const TVector3 & referencePoint) const {
    jacobian.block< 3, 3 >(0, 0) = cof.R;
    jacobian.block< 3, 3 >(0, 3).setZero();
    jacobian.block< 3, 1 >(3, 0) = cof.R.col(0).cross(referencePoint - cof.r);
    jacobian.block< 3, 1 >(3, 1) = cof.R.col(1).cross(referencePoint - cof.r);
    jacobian.block< 3, 1 >(3, 2) = cof.R.col(2).cross(referencePoint - cof.r);
    jacobian.block< 3, 3 >(3, 3) = cof.R;
}

void FreeBase::integrate(TTime dt) {

    cof.r += cof.v * dt;
    cof.v += cof.dotv * dt;

    TScalar angle = cof.omega.norm();
    if (angle != 0) {
        cof.R = cof.R * Eigen::AngleAxis< TScalar >((angle * dt), cof.omega / angle);
    }

    cof.omega += cof.dotomega * dt;
}

void FreeBase::storeState() {
    storedCof = cof;
}

void FreeBase::restoreState() {
    cof = storedCof;
}

TVectorX FreeBase::getState() const {
    assert(0);
    return TVectorX();
}
TVectorX FreeBase::getDStateDt() const {
    assert(0);
    return TVectorX();
}

// should be 6 when implemented
size_t FreeBase::getNumberOfStateVariables() const {
    return 0;
}
void FreeBase::setState(const TVectorX & state) {
    assert(0);
}

void FreeBase::calcRelPose(TVector3 & /*relr*/, TMatrix3x3 & /*relR*/) {
}
