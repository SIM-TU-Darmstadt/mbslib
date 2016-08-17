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
 * \file mbslib/elements/joint/RevoluteJoint.cpp
 * Definition of mbslib::RevoluteJoint
 */

#include <Eigen/Geometry>
#include <iostream>
#include <math.h>
#include <mbslib/elements/joint/RevoluteJoint.hpp>
#include <mbslib/utility/internalTests.hpp>
#include <mbslib/utility/mathtools.hpp>

using namespace mbslib;

RevoluteJoint::RevoluteJoint(MbsObject & pred, const TVector3 & axis, int jointId, int stateVectorPosition, TScalar jointOffset, TScalar jointFriction, TScalar gearRatio, TScalar rotorInertia, const std::string & name)
    : Joint1DOF(pred, jointId, stateVectorPosition, jointOffset, jointFriction, gearRatio, rotorInertia, name) {
    this->axis = axis;
    if (this->axis.norm() == 0) {
        setupError = true;
    } else {
        this->axis.normalize();
    }
    checkUnitVector(this->axis);
    S.segment< 3 >(0) = this->axis;
    S.segment< 3 >(3).setZero();
}

void RevoluteJoint::doVelocity() {
    /*if (!checkUnitVector(axis)) {
        std::cout << "joint '" << getName() << "': checkUnitVector on axis failed" << std::endl;
    }*/
    cof.omega = getRelativeOrientation().transpose() * pred->getCoordinateFrame().omega + axis * jp.dotq;
    cof.v = getRelativeOrientation().transpose() * pred->getCoordinateFrame().v;
}

void RevoluteJoint::doAcceleration() {
    checkUnitVector(axis);
    cof.dotomega = getRelativeOrientation().transpose() * pred->getCoordinateFrame().dotomega + axis * jp.ddotq + cof.omega.cross(axis * jp.dotq);
    cof.dotv = getRelativeOrientation().transpose() * pred->getCoordinateFrame().dotv;
}

void RevoluteJoint::doRneInward(bool withExternalForce) {
    checkUnitVector(axis);
    rneForceTorque.f = getRelativeOrientation() * succ->getRneForceTorque().f;
    rneForceTorque.n = getRelativeOrientation() * succ->getRneForceTorque().n;
    tau = axis.dot(rneForceTorque.n) + jp.dotq * jointFriction + squaredGearRotorInertia * jp.ddotq + (withExternalForce ? (-externalTau) : (static_cast< TScalar >(0)));
}

void RevoluteJoint::doAggregateBody() {
    aggregateBody.m = succ->getAggregateBody().m;
    aggregateBody.I = getRelativeOrientation() * succ->getAggregateBody().I * getRelativeOrientation().transpose();
    aggregateBody.com = getRelativeOrientation() * succ->getAggregateBody().com;
}

TScalar RevoluteJoint::calculateCRBAforce(TVector3 & f, TVector3 & n) {
    // This are force and torque resulting from accelerating
    // the aggregate body at the given joint
    // force and torque are given in world-coordinates
    const AggregateBody & ab = succ->getAggregateBody();
    TVector3 ax = axis;
    f = ax.cross(ab.com) * ab.m;
    n = ab.com.cross(f) + ab.I * ax;
    //std::cout << ab.m << " " << ab.I << "  " << ax.transpose() << "  " << ab.com.transpose() << "  " << f.transpose()<< "  " << n.transpose() << std::endl;
    TScalar tau = n.dot(ax);
    f = cof.R * f;
    n = cof.R * n;
    return tau + squaredGearRotorInertia; // * jp.ddotq;
}

TScalar RevoluteJoint::calculateCRBAforce(const TVector3 & pos, const TVector3 & f, const TVector3 & n) {
    //TVector3 flocal = cof.R.transpose() * f;
    TVector3 nlocal = cof.R.transpose() * (n + (pos - cof.r).cross(f));
    //std::cout << pos.transpose() - cof.r.transpose() << "  " << f.transpose() << std::endl;
    return nlocal.dot(axis);
}

void RevoluteJoint::doABASweep1fwd() {
    cof.spatialVelocity = makeVelocityTransform(TMatrix3x3(getRelativeOrientation().transpose())) * pred->getCoordinateFrame().spatialVelocity + S * jp.dotq;

    c = spatialCross(cof.spatialVelocity, S * jp.dotq);
    // we only need to calculated c, as it is used in further sweeps
    // we can ommit the cj part, as S_circ is zero
    // so that c = v x vj = [ omega ; v ] x [ omega_rel ; 0 ] with omega_rel = axis * qdot

    // we can ommit p, as I is zero and there is no external force

    //std::cout << "revj - v = " << cof.spatialVelocity.transpose() << " c = " << c.transpose() <<  std::endl;
}

void RevoluteJoint::doABASweep1inv() {
    cof.spatialVelocity = makeVelocityTransform(TMatrix3x3(getRelativeOrientation().transpose())) * pred->getCoordinateFrame().spatialVelocity + S * jp.dotq;

    c = spatialCross(cof.spatialVelocity, S * jp.dotq) + S * jp.ddotq;
    // we can ommit p, as I is zero and there is no external force

    //std::cout << "revj - v = " << cof.spatialVelocity.transpose() << " c = " << c.transpose() <<  std::endl;
}

void RevoluteJoint::doABASweep2fwd() {
    const TMatrix6x6 & IA = succ->getArticulatedBodyInertia();
    const TVector6 & pA = succ->getBiasForce();

    checkUnitVector(S);
    U = IA * S;
    D(0, 0) = S.dot(U) + squaredGearRotorInertia; // S.transpose() * U;
    //std::cout << "revj IA = " << std::endl << IA << std::endl << "U = " << U.transpose() << " S = " << S.transpose() << " D(0,0) = " << D(0,0) << std::endl;
    //assert(!std::isnan(D(0,0)));
    checkNotZero(D(0, 0));
    //assert(externalTau == 0);
    //assert(jointFriction == 0);
    u(0) = tau + externalTau - (jp.dotq * jointFriction) - S.dot(pA);

    TMatrix6x6 abiAtEndOfLink = (IA - U * (1 / D(0, 0)) * U.transpose());
    /*
  articulatedBodyInertia = makeForceTransform(TMatrix3x3(getRelativeOrientation())) * abiAtEndOfLink * makeVelocityTransform(TMatrix3x3(getRelativeOrientation().transpose()));
  biasForce = makeForceTransform( TMatrix3x3(getRelativeOrientation()) ) * ( pA + abiAtEndOfLink * c + U * (1/D(0,0)) * u );
  */
    transformABI(articulatedBodyInertia, getRelativeOrientation(), abiAtEndOfLink);
    transformForce(biasForce, getRelativeOrientation(), (pA + abiAtEndOfLink * c + U * (1 / D(0, 0)) * u));

    //std::cout << "revj - s2 ABI = " << std::endl << articulatedBodyInertia << std::endl << " biasForce = " << biasForce.transpose() << std::endl;
}

void RevoluteJoint::doABASweep2inv() {
    const TMatrix6x6 & IA = succ->getArticulatedBodyInertia();
    const TVector6 & pA = succ->getBiasForce();

    //TMatrix6x6 abiAtEndOfLink = IA;
    articulatedBodyInertia = makeForceTransform(TMatrix3x3(getRelativeOrientation())) * IA * makeVelocityTransform(TMatrix3x3(getRelativeOrientation().transpose()));
    biasForce = makeForceTransform(TMatrix3x3(getRelativeOrientation())) * (pA + IA * c);
    //std::cout << "revj - s2 ABI = " << std::endl << articulatedBodyInertia << std::endl << " biasForce = " << biasForce.transpose() << std::endl;
}

void RevoluteJoint::doABASweep3fwd() {
    TMatrix3x3 Rinv = getRelativeOrientation().transpose();
    TVector6 atmp = makeVelocityTransform(Rinv) * pred->getCoordinateFrame().spatialAcceleration + c;
    jp.ddotq = (1 / D(0, 0)) * (u(0) - U.dot(atmp));
    //assert(!std::isnan(jp.ddotq));
    checkUnitVector(S);
    cof.spatialAcceleration = atmp + S * jp.ddotq;
    //std::cout << "revj -- atmp = " << atmp.transpose() << " c = " << c.transpose() << " apred = " << pred->getCoordinateFrame().spatialAcceleration.transpose() << std::endl;
}

void RevoluteJoint::doABASweep3inv() {
    TMatrix3x3 Rinv = getRelativeOrientation().transpose();
    TVector6 atmp = makeVelocityTransform(Rinv) * pred->getCoordinateFrame().spatialAcceleration + c;

    checkUnitVector(S);

    tau = S.dot(succ->getArticulatedBodyInertia() * atmp + succ->getBiasForce());
    cof.spatialAcceleration = atmp;
    //std::cout << "revj -- atmp = " << atmp.transpose() << " c = " << c.transpose() << " apred = " << pred->getCoordinateFrame().spatialAcceleration.transpose() << std::endl;
}

void RevoluteJoint::makeJacobian(TMatrix6xX & jacobian, const TVector3 & referencePoint) const {
    jacobian.block< 3, 1 >(0, stateVectorPosition) = cof.R * axis;
    jacobian.block< 3, 1 >(3, stateVectorPosition) = (cof.R * axis).cross(referencePoint - cof.r);
}

void RevoluteJoint::calcRelPose(TVector3 & relr, TMatrix3x3 & relR) {
    //const CoordinateFrame & predCof = pred->getCoordinateFrame();
    //cof.r = predCof.r;
    relR = Eigen::AngleAxis< TScalar >((jp.q + jointOffset), axis);
    //cof.R = predCof.R * getRelativeOrientation();
}

bool RevoluteJoint::getIsStateVariableUnconstrainedAngle(unsigned int /*i*/) {
    // the first
    //return i==0;
    return false;
}

const TVector3 & RevoluteJoint::getAxis() const {
    return this->axis;
}

RevoluteJoint::~RevoluteJoint() {
}
