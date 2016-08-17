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
 * \file mbslib/elements/joint/Joint1DOF.cpp
 * Definition of mbslib::Joint1DOF
 */

#include <float.h>
#include <iostream>
#include <mbslib/elements/joint/Joint1DOF.hpp>
#include <mbslib/elements/joint/JointNDOF.hpp>
#include <mbslib/utility/types.hpp>

using namespace mbslib;

Joint1DOF::Joint1DOF(MbsObject & pred, int jointId, int stateVectorPosition, TScalar jointOffset, TScalar jointFriction, TScalar gearRatio, TScalar rotorInertia, const std::string & name)
    : JointNDOF< 1 >(pred, jointId, stateVectorPosition, name)
    , lowerPositionLimit(-DBL_MAX)
    , upperPositionLimit(DBL_MAX)
    , effortLimit(DBL_MAX)
    , velocityLimit(DBL_MAX)
    , externalTau(0)
    , jointFriction(jointFriction)
    , gearRatio(gearRatio)
    , rotorInertia(rotorInertia)
    , squaredGearRotorInertia(gearRatio * gearRatio * rotorInertia) {
    jp.q = 0;
    jp.dotq = 0;
    jp.ddotq = 0;
    storedJP.q = 0;
    storedJP.dotq = 0;
    storedJP.ddotq = 0;
    this->jointOffset = jointOffset;
    tau = 0;
}
//IIntegrator<TScalar> & integrator =  IntegratorFactory::create<TScalar>();
//IIntegrator<TScalar> & integrator2 = IntegratorFactory::create<TScalar>();

Joint1DOF::~Joint1DOF() {
}
void Joint1DOF::integrate(TTime dt) {
    /*integrator.setX(jp.q);
  jp.q = integrator.integrate(dt,jp.dotq,&(jp.ddotq));

  integrator2.setX(jp.dotq);
  jp.dotq = integrator2.integrate(dt,jp.ddotq);*/

    jp.q += jp.dotq * dt;
    jp.dotq += jp.ddotq * dt;

    enforceLimits();
}

void Joint1DOF::enforceLimits() {
    if (jp.q < getJointLowerPositionLimit()) {
        jp.q = getJointLowerPositionLimit();
        jp.dotq = 0;
        std::cout << "Warning: Joint Lower Position Limit hit non physical behaviour ensuing" << std::endl;
    }
    if (jp.q > getJointUpperPositionLimit()) {
        jp.q = getJointUpperPositionLimit();
        jp.dotq = 0;
        std::cout << "Warning: Joint Upper Position Limit hit non physical behaviour ensuing" << std::endl;
    }
    if (jp.dotq > getJointVelocityLimit()) {
        jp.dotq = getJointVelocityLimit();
        std::cout << "Warning: Joint Velocity Limit hit non physical behaviour ensuing" << std::endl;
    }
    if (jp.dotq < -getJointVelocityLimit()) {
        jp.dotq = -getJointVelocityLimit();
        std::cout << "Warning: Joint Velocity Limit hit non physical behaviour ensuing" << std::endl;
    }
    //TODO JOINT EFFORT LIMIT
}

void Joint1DOF::storeState() {
    storedJP = jp;
}

void Joint1DOF::restoreState() {
    jp = storedJP;
}

TVectorX Joint1DOF::getState() const {
    return ((TVectorX(2) << jp.q, jp.dotq).finished());
}

TVectorX Joint1DOF::getDStateDt() const {
    return ((TVectorX(2) << jp.dotq, jp.ddotq).finished());
}

void Joint1DOF::setState(const TVectorX & v) {
    assert(v.size() == 2);
    jp.q = v(0);
    jp.dotq = v(1);

    enforceLimits();
}

const JointState & Joint1DOF::getJointState() const {
    return jp;
}

/**
 * \brief Get joint state.
 *
 * \return  The joint state jp.
 */
JointState & Joint1DOF::getJointState() {
    return jp;
}

/**
 * \brief Get the joint position.
 *
 * \return The joint position q.
 */
TScalar Joint1DOF::getJointPosition() {
    return jp.q;
}

/**
 * \brief Get the joint velocity.
 *
 * return The joint velocity dotq.
 */
TScalar Joint1DOF::getJointVelocity() {
    return jp.dotq;
}

/**
 * \brief Get the joint acceleration.
 *
 * \return The joint acceleration ddotq.
 */
TScalar Joint1DOF::getJointAcceleration() {
    return jp.ddotq;
}

/**
 * \brief Get the joints lower limit.
 *
 */
TScalar Joint1DOF::getJointLowerPositionLimit() const {
    return lowerPositionLimit;
}
/**
 * \brief Gets the joints upper limit.
 *
 */
TScalar Joint1DOF::getJointUpperPositionLimit() const {
    return upperPositionLimit;
}
/**
 * \brief Sets the joints lower limit.
 *
 */
void Joint1DOF::setJointLowerPositionLimit(TScalar value) {
    lowerPositionLimit = value;
}

/**
 * \brief Sets the joints upper limit.
 *
 */
void Joint1DOF::setJointUpperPositionLimit(TScalar value) {
    upperPositionLimit = value;
}

void Joint1DOF::setJointVelocityLimit(TScalar value) {
    velocityLimit = value;
}
void Joint1DOF::setJointEffortLimit(TScalar value) {
    effortLimit = value;
}
TScalar Joint1DOF::getJointVelocityLimit() const {
    return velocityLimit;
}
TScalar Joint1DOF::getJointEffortLimit() const {
    return effortLimit;
}

void Joint1DOF::getLowerStateLimits(TVectorX & limits) const {
    limits.resize(2);
    limits(0) = getJointLowerPositionLimit();
    limits(1) = -getJointVelocityLimit();
}
void Joint1DOF::getUpperStateLimits(TVectorX & limits) const {
    limits.resize(2);
    limits(0) = getJointUpperPositionLimit();
    limits(1) = getJointVelocityLimit();
}

std::string Joint1DOF::getStateVariableName(unsigned int i) const {
    std::stringstream ss;
    ss << getName();
    if (i == 0) {

        ss << " q(t)";
    } else {
        ss << " q'(t)";
    }
    return ss.str();
}

/**
 * \brief Set the joint position.
 *
 * \param q The joint position q.
 */
void Joint1DOF::setJointPosition(TScalar q) {
    jp.q = q;
}

/**
 * \brief Set the joint velocity.
 *
 * \param dotq The joint velocity dotq.
 */
void Joint1DOF::setJointVelocity(TScalar dotq) {
    jp.dotq = dotq;
}

/**
 * \brief Set the joint acceleration.
 *
 * \param ddotq The joint acceleration ddotq.
 */
void Joint1DOF::setJointAcceleration(TScalar ddotq) {
    jp.ddotq = ddotq;
}

/**
 * \brief Get force of joint.
 *
 * \return  The joint force|torque.
 */
TScalar Joint1DOF::getJointForceTorque() const {
    return tau;
}

/**
 * \brief Set force of joint.
 *
 * \param tau_joint The force at joint.
 */
void Joint1DOF::setJointForceTorque(TScalar tau_joint) {
    tau = tau_joint;
}

/**
 * \brief Get force of joint.
 *
 * \return The external joint force torque.
 */
TScalar Joint1DOF::getExternalJointForceTorque() const {
    return externalTau;
}

/**
 * \brief Set force of joint.
 *
 * \param tau_joint The joint force.
 */
void Joint1DOF::setExternalJointForceTorque(TScalar tau_joint) {
    externalTau = tau_joint;
}

/**
 * \brief Get number of state variables.
 *
 * \return  The number of state variables.
 */
size_t Joint1DOF::getNumberOfStateVariables() const {
    return 2;
}

/**
 * \brief Aba set tau.
 *
 * \param t \todo kommentieren.
 */
void Joint1DOF::abaSetTau(const TVectorN & t) {
    tau = t(0);
}

/**
 * \brief Aba set dot q.
 *
 * \param dotq  \todo kommentieren.
 */
void Joint1DOF::abaSetDotQ(const TVectorN & dotq) {
    jp.dotq = dotq(0);
}

/**
 * \brief Aba set d dot q.
 *
 * \param ddotq \todo kommentieren
 */
void Joint1DOF::abaSetDDotQ(const TVectorN & ddotq) {
    jp.ddotq = ddotq(0);
}
