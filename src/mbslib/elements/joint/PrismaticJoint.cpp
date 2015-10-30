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
 * \file mbslib/elements/joint/PrismaticJoint.cpp
 * Definition of mbslib::PrismaticJoint
 */

#include <mbslib/elements/joint/PrismaticJoint.hpp>
#include <mbslib/utility/mathtools.hpp>
#include <mbslib/utility/internalTests.hpp>
#include <Eigen/Geometry>
#include <math.h>

using namespace mbslib;

PrismaticJoint::PrismaticJoint(MbsObject & pred, const TVector3 & direction, int jointId, int stateVectorPosition, TScalar jointOffset, TScalar jointFriction, TScalar gearRatio, TScalar rotorInertia, const std::string & name)
    : Joint1DOF(pred, jointId, stateVectorPosition, jointOffset, jointFriction, gearRatio, rotorInertia, name) {
    dir = direction;
    if (dir.norm() == 0) {
        setupError = true;
    } else {
        dir.normalize();
    }
    //assert(dir.norm() == 1);
    checkUnitVector(dir);
    S.segment< 3 >(0).setZero();
    S.segment< 3 >(3) = dir;
}

TScalar PrismaticJoint::calculateCRBAforce(TVector3 & f, TVector3 & n) {
    const AggregateBody & ab = succ->getAggregateBody();
    TVector3 ax = dir;
    f = ax * ab.m;
    n = ab.com.cross(f);
    //std::cout << ab.m << " " << ab.I << "  " << ax.transpose() << "  " << ab.com.transpose() << "  " << f.transpose()<< "  " << n.transpose() << std::endl;
    TScalar tau = f.dot(ax);
    f = cof.R * f;
    n = cof.R * n;
    return tau + squaredGearRotorInertia;
}

TScalar PrismaticJoint::calculateCRBAforce(const TVector3 & pos, const TVector3 & f, const TVector3 & n) {
    TVector3 flocal = cof.R.transpose() * f;
    return flocal.dot(dir);
}

void PrismaticJoint::doVelocity() {
    checkUnitVector(dir);
    cof.omega = pred->getCoordinateFrame().omega;
    cof.v = pred->getCoordinateFrame().v + cof.omega.cross(dir * jp.q) + dir * jp.dotq;
}

void PrismaticJoint::doAcceleration() {
    checkUnitVector(dir);
    cof.dotomega = pred->getCoordinateFrame().dotomega;
    cof.dotv = pred->getCoordinateFrame().dotv + cof.dotomega.cross(dir * jp.q) + cof.omega.cross(cof.omega.cross(dir * (jp.q + jointOffset))) + 2 * cof.omega.cross(dir * jp.dotq) + dir * jp.ddotq;
}

void PrismaticJoint::doRneInward(bool withExternalForce) {
    checkUnitVector(dir);
    rneForceTorque.f = succ->getRneForceTorque().f;
    rneForceTorque.n = succ->getRneForceTorque().n + (dir * (jp.q + jointOffset)).cross(rneForceTorque.f);
    tau = dir.dot(rneForceTorque.f) + jp.dotq * jointFriction + squaredGearRotorInertia * jp.ddotq + (withExternalForce ? (-externalTau) : (static_cast< TScalar >(0)));
}

void PrismaticJoint::doAggregateBody() {
    aggregateBody.m = succ->getAggregateBody().m;
    aggregateBody.I = succ->getAggregateBody().I;
    aggregateBody.com = succ->getAggregateBody().com + (dir * (jp.q + jointOffset));
}

void PrismaticJoint::doABASweep1fwd() {
    cof.spatialVelocity = makeVelocityTransform(TVector3(-dir * jp.q)) * pred->getCoordinateFrame().spatialVelocity + S * jp.dotq;

    // we only need to calculated c, as it is used in further sweeps
    // we can ommit the cj part, as S_circ is zero
    // so that c = v x vj = [ omega ; v ] x [ 0 ; vrel ] with vrel = axis * qdot

    c = spatialCross(cof.spatialVelocity, S * jp.dotq);
    //TODO: check the more efficient implementation for c:
    //c.segment<3>(0).setZero(); // = omega x 0 + 0 x vrel
    //c.segment<3>(3) = cof.spatialVelocity.segment<3>(0).cross(dir * jp.dotq); // = v x 0 + omega x vrel

    // std::cout << "prisj - s1 vel = " << cof.spatialVelocity.transpose() << " c = " << c.transpose() << std::endl;
    // we can ommit p, as I is zero and there is no external force
}

void PrismaticJoint::doABASweep1inv() {
    //doPosition();
    cof.spatialVelocity = makeVelocityTransform(TVector3(-dir * jp.q)) * pred->getCoordinateFrame().spatialVelocity + S * jp.dotq;

    // we only need to calculated c, as it is used in further sweeps
    // we can ommit the cj part, as S_circ is zero
    // so that c = v x vj = [ omega ; v ] x [ 0 ; vrel ] with vrel = axis * qdot

    c = spatialCross(cof.spatialVelocity, S * jp.dotq) + S * jp.ddotq;
    //TODO: check the more efficient implementation for c:
    //c.segment<3>(0).setZero(); // = omega x 0 + 0 x vrel
    //c.segment<3>(3) = cof.spatialVelocity.segment<3>(0).cross(dir * jp.dotq); // = v x 0 + omega x vrel

    // std::cout << "prisj - s1 vel = " << cof.spatialVelocity.transpose() << " c = " << c.transpose() << std::endl;
    // we can ommit p, as I is zero and there is no external force
}

void PrismaticJoint::doABASweep2fwd() {
    checkUnitVector(S);

    // some joint specialities are still forbidden to use :-)
    assert(squaredGearRotorInertia == 0);
    //assert(externalTau == 0);
    assert(jointFriction == 0);

    const TMatrix6x6 & IA = succ->getArticulatedBodyInertia();
    const TVector6 & pA = succ->getBiasForce();

    // std::cout << "prisj - sweep2 S = " << S.transpose() << " pA = " << pA.transpose() << std::endl;

    U = IA * S;
    D(0, 0) = S.dot(U) + squaredGearRotorInertia; // S.transpose() * U;
    checkNotZero(D(0, 0));
    u(0) = tau + externalTau - (jp.dotq * jointFriction) - S.dot(pA);

    TMatrix6x6 abiAtEndOfLink = (IA - U * (1 / D(0, 0)) * U.transpose());

    transformABI(articulatedBodyInertia, getRelativePosition(), abiAtEndOfLink);
    transformForce(biasForce, getRelativePosition(), (pA + abiAtEndOfLink * c + U * (1 / D(0, 0)) * u));
}

void PrismaticJoint::doABASweep2inv() {
    checkUnitVector(S);

    // some joint specialities are still forbidden to use :-)
    assert(squaredGearRotorInertia == 0);
    assert(externalTau == 0);
    assert(jointFriction == 0);

    const TMatrix6x6 & IA = succ->getArticulatedBodyInertia();
    const TVector6 & pA = succ->getBiasForce();

    // std::cout << "prisj - sweep2 S = " << S.transpose() << " pA = " << pA.transpose() << std::endl;

    //TMatrix6x6 abiAtEndOfLink = IA;
    articulatedBodyInertia = makeForceTransform(getRelativePosition()) * IA * makeVelocityTransform(TVector3(-1 * getRelativePosition()));
    biasForce = makeForceTransform(getRelativePosition()) * (pA + IA * c);
}

void PrismaticJoint::doABASweep3fwd() {
    TVector6 atmp = makeVelocityTransform(TVector3(-1 * getRelativePosition())) * pred->getCoordinateFrame().spatialAcceleration + c;

    //std::cout << "prisj - sweep3 a' = " << atmp.transpose() << " U = " << U.transpose() << " u = " << u(0) << " D = " << D(0,0) << std::endl;
    jp.ddotq = (1 / D(0, 0)) * (u(0) - U.dot(atmp));
    //assert(!std::isnan(jp.ddotq));
    checkUnitVector(S);
    cof.spatialAcceleration = atmp + S * jp.ddotq;
    //std::cout << "prisj -- atmp = " << atmp.transpose() << " c = " << c.transpose() << std::endl;
}

void PrismaticJoint::doABASweep3inv() {
    TVector6 atmp = makeVelocityTransform(TVector3(-1 * getRelativePosition())) * pred->getCoordinateFrame().spatialAcceleration + c;

    //std::cout << "prisj inv atmp = " << atmp.transpose() << " S = " << S.transpose() << " p = " << succ->getBiasForce().transpose() << std::endl << "ABI = " << succ->getArticulatedBodyInertia() << std::endl << std::endl;

    tau = S.dot(succ->getArticulatedBodyInertia() * atmp + succ->getBiasForce());
    checkUnitVector(S);
    cof.spatialAcceleration = atmp;
    //std::cout << "prisj -- atmp = " << atmp.transpose() << " c = " << c.transpose() << std::endl;
}

void PrismaticJoint::makeJacobian(TMatrix6xX & jacobian, const TVector3 & referencePoint) const {
    jacobian.block< 3, 1 >(0, stateVectorPosition).setZero();
    jacobian.block< 3, 1 >(3, stateVectorPosition) = cof.R * dir;
}

void PrismaticJoint::calcRelPose(TVector3 & relr, TMatrix3x3 & relR) {
    checkUnitVector(dir);
    //const CoordinateFrame & predCof = pred->getCoordinateFrame();
    //cof.R = predCof.R;
    assert(jointOffset == 0);
    relr = dir * (jp.q + jointOffset);
    //cof.r = predCof.r + cof.R * ( relr );
}
const TVector3 & PrismaticJoint::getDirection() const {
    return dir;
}
PrismaticJoint::~PrismaticJoint() {
}
