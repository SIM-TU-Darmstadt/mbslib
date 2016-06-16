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
 * \file mbslib/elements/rigidbody/RigidLink.cpp
 * Definition of mbslib::RigidLink
 */

#include <mbslib/elements/rigidbody/RigidLink.hpp>
#include <mbslib/utility/mathtools.hpp>
#include <mbslib/utility/internalTests.hpp>
#include <Eigen/Geometry>

using namespace mbslib;

RigidLink::RigidLink(MbsObject & pred, const TVector3 & relr, const TVector3 & com, TScalar m, const TMatrix3x3 & I, const std::string & name)
    : MbsObjectOneSucc(pred, name)
    , p(Eigen::Matrix< TScalar, 6, 1 >::Zero())
    , U(Eigen::Matrix< TScalar, 6, 1 >::Zero())
    , D(0)
    , u(0) {
    checkInertia(m, I);

    this->fixedRelr = relr;
    this->com = com;
    this->m = m;
    this->I = I;

    //create ABA-Inertia
    updateABA();
}

RigidLink::RigidLink(MbsObject & pred, const RigidBodyDescription & rigidBodyDescription, const std::string & name)
    : MbsObjectOneSucc(pred, name) {

    this->fixedRelr = rigidBodyDescription.r;
    this->com = rigidBodyDescription.com;
    this->m = rigidBodyDescription.m;
    this->I = rigidBodyDescription.I;

    checkInertia(m, I);

    //create ABA-Inertia
    updateABA();
}

RigidLink::~RigidLink() {
}

void RigidLink::doVelocity() {
    cof.omega = pred->getCoordinateFrame().omega;
    cof.v = pred->getCoordinateFrame().v + cof.omega.cross(this->getRelativePosition());
}

void RigidLink::doAcceleration() {
    cof.dotomega = pred->getCoordinateFrame().dotomega;
    cof.dotv = pred->getCoordinateFrame().dotv + cof.dotomega.cross(this->getRelativePosition()) + cof.omega.cross(cof.omega.cross(this->getRelativePosition()));
}

void RigidLink::doRneInward(bool withExternalForce) {

    TVector3 acom = cof.dotv + cof.dotomega.cross(this->com) + cof.omega.cross(cof.omega.cross(this->com));
    TVector3 F = acom * m;
    TVector3 bla = I * cof.omega;
    TVector3 N = I * cof.dotomega + cof.omega.cross(bla);

    rneForceTorque.f = succ->getRneForceTorque().f + F;
    bla = getRelativePosition() + com;
    rneForceTorque.n = succ->getRneForceTorque().n + getRelativePosition().cross(succ->getRneForceTorque().f) + bla.cross(F) + N;
}

void RigidLink::doAggregateBody() {
    TScalar m1 = m;
    const TVector3 com1 = getRelativePosition() + com;
    const TMatrix3x3 & I1 = I;
    TScalar m2 = succ->getAggregateBody().m;
    const TVector3 com2 = getRelativePosition() + succ->getAggregateBody().com;
    const TMatrix3x3 & I2 = succ->getAggregateBody().I;

    if (m1 == 0) {
        aggregateBody.m = m2;
        aggregateBody.com = com2;
        aggregateBody.I = I2;
        return;
    }

    TVector3 ccom = (m1 * com1 + m2 * com2) / (m1 + m2);
    TVector3 dcom1 = com1 - ccom;
    TVector3 dcom2 = com2 - ccom;

    aggregateBody.m = m1 + m2;
    aggregateBody.com = ccom;
    aggregateBody.I = I1 + m1 * (dcom1.dot(dcom1) * TMatrix3x3::Identity() - dcom1 * dcom1.transpose()) + I2 + m2 * (dcom2.dot(dcom2) * TMatrix3x3::Identity() - dcom2 * dcom2.transpose());

    checkInertia(aggregateBody.m, aggregateBody.I);
}

void RigidLink::doABASweep1fwd() {
    // recalculate ABI for each interation to enable parameter estimation of m/com/I
    TMatrix3x3 dispCross = makeCrossproductMatrix(fixedRelr + com);
    abaI.block< 3, 3 >(0, 0) = I + m * dispCross * dispCross.transpose();
    abaI.block< 3, 3 >(3, 0) = m * dispCross.transpose();
    abaI.block< 3, 3 >(0, 3) = m * dispCross;
    abaI.block< 3, 3 >(3, 3) = TMatrix3x3::Identity() * m;

    cof.spatialVelocity = makeVelocityTransform(TVector3(-getRelativePosition())) * pred->getCoordinateFrame().spatialVelocity;

    // calculate bias force required at center of mass;
    // we have to translate the velocity to the proper position!
    // at the com the 6x6 Inertia is [I 0;0 m*1]
    p = spatialCrossStar(pred->getCoordinateFrame().spatialVelocity, abaI * pred->getCoordinateFrame().spatialVelocity);
    //assert(0);

    //std::cout << "rigl - v = " << cof.spatialVelocity.transpose() << " p = " << p.transpose() << std::endl;
}

void RigidLink::doABASweep2fwd() {
    // the ABI is quite simple; we take the inertia of this body and the ABI from the successor
    // translate both to the beginning  of the link and sum up there
    // luckily the inertia of the link is already translated there

    transformABI(articulatedBodyInertia, getRelativePosition(), succ->getArticulatedBodyInertia());
    articulatedBodyInertia += abaI;
    transformForce(biasForce, getRelativePosition(), succ->getBiasForce());
    biasForce += p;

    /*
  std::cout << "rl -   p = " << p.transpose() << std::endl;
  std::cout << "rl - vel = " << cof.spatialVelocity.transpose() << std::endl;
  std::cout << "rl - acc = " << cof.spatialAcceleration.transpose() << std::endl;
  std::cout << "rl - biasForce = " << biasForce.transpose() << std::endl;
  std::cout << "rl - abaI = " << std::endl << abaI <<std::endl;
  std::cout << "rl - abi = " << std::endl << articulatedBodyInertia << std::endl;
  */

    // currently I'm not quite sure, if we have to consider the local inertia for the bias force,
    // so this is postponed for later
    //assert(0);
}

void RigidLink::doABASweep3fwd() {
    TVector6 atmp = makeVelocityTransform(TVector3(-1 * getRelativePosition())) * pred->getCoordinateFrame().spatialAcceleration;
    cof.spatialAcceleration = atmp;
    //assert(0);
}

void RigidLink::calcRelPose(TVector3 & relr, TMatrix3x3 & relR) {
    //cof.R = pred->getCoordinateFrame().R;
    //cof.r = pred->getCoordinateFrame().r + cof.R * relr;
    relr = fixedRelr;
}

int RigidLink::getDOF() const {
    return 0;
}

/**
 * \brief First (outward) sweep of inverse dynamics ABA.
 */
void RigidLink::doABASweep1inv() {
    doABASweep1fwd();
}

/**
 * \brief Second (inward) sweep of inverse dynamics ABA.
 */
void RigidLink::doABASweep2inv() {
    doABASweep2fwd();
}

/**
 * \brief Third (outward) sweep of inverse dynamics ABA.
 */
void RigidLink::doABASweep3inv() {
    doABASweep3fwd();
}

/**
 * \brief Make columns of jacobian for a given position.
 *
 * \param [out]     jacobian  The jacobian.
 * \param referencePoint      The reference point.
 */
void RigidLink::makeJacobian(TMatrix6xX & /*jacobian*/, const TVector3 & /*referencePoint*/) const {
    return;
}

/**
 * \brief returns the mass of this rigid link.
 *
 * \return  The mass.
 */
const TScalar & RigidLink::getMass() const {
    return this->m;
}

/**
 * \brief returns the inertia tensor of this rigid link.
 *
 * \return  The inertia tensor.
 */
const TMatrix3x3 & RigidLink::getInertiaTensor() const {
    return this->I;
}

/**
 * \brief returns the center of mass vector.
 *
 * \return  The center of mass.
 */
const TVector3 & RigidLink::getCenterOfMass() const {
    return this->com;
}

/**
 * \brief returns the relativ position.
 *
 * \return  The fixed relative position.
 */
const TVector3 & RigidLink::getFixedRelativePosition() const {
    return this->fixedRelr;
}

RigidLink& RigidLink::operator=(const RigidBodyDescription& rigidBodyDescription) {
    this->com = rigidBodyDescription.com;
    this->fixedRelr = rigidBodyDescription.r;
    this->I = rigidBodyDescription.I;
    this->m = rigidBodyDescription.m;
    updateABA();
    return *this;
}

void RigidLink::setCenterOfMass(const TVector3 & com) {
    this->com = com;
    updateABA();
}

void RigidLink::setFixedRelativePosition(const TVector3 &relr) {
    this->fixedRelr = relr;
    updateABA();
}

void RigidLink::setInertiaTensor(const TMatrix3x3 &I) {
    this->I = I;
    updateABA();
}

void RigidLink::setMass(const TScalar & m) {
    this->m = m;
    updateABA();
}

void RigidLink::updateABA() {
    ///create ABA-Inertia
    ///at the beginning of the link, not at the CoF!!!
    ///in a parameter-optimization scenario, this calculation must be done in each step, as parameters may change
    TMatrix3x3 dispCross = makeCrossproductMatrix(fixedRelr + com);
    abaI.block< 3, 3 >(0, 0) = I + m * dispCross * dispCross.transpose();
    abaI.block< 3, 3 >(3, 0) = m * dispCross.transpose();
    abaI.block< 3, 3 >(0, 3) = m * dispCross;
    abaI.block< 3, 3 >(3, 3) = TMatrix3x3::Identity() * m;
}
