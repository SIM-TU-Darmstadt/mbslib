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
 * \file mbslib/elements/compound/MbsCompound.cpp
 * Definition of mbslib::MbsCompound
 */

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <mbslib/elements/MbsCompound.hpp>
#include <mbslib/elements/base/FixedBase.hpp>
#include <mbslib/elements/base/FreeBase.hpp>
#include <mbslib/utility/internalTests.hpp>
#include <mbslib/utility/mathtools.hpp>

using namespace mbslib;

MbsCompound::MbsCompound(const std::string & name_)
    : ParametrizedObjectCollection(name_)
    , dof(0)
    , base(NULL)
    , freeBase(false)
    , endpoint(NULL)
    , statesOfJoints(0)
    , statesOfNonJoints(0)
    , numberOfControlValues(0)
    , gravitationVector() {
    gravitationVector.setZero();
}

MbsCompound::~MbsCompound() {
    for (MbsObject * element : elements) {
        delete element;
    }
}

bool MbsCompound::isValid() const {
    if (base == nullptr) {
        return false;
    }
    if (endpoint == nullptr) {
        return false;
    }
    for (MbsObject * element : elements) {
        if (!element->isValid()) {
            return false;
        }
    }
    return true;
}

bool MbsCompound::hasBase() const {
    return (base != NULL);
}

Base & MbsCompound::getBase() {
    return *base;
}

const Base & MbsCompound::getBase() const {
    return *base;
}

Endpoint & MbsCompound::getEnd() {
    return *endpoint;
}

const Endpoint & MbsCompound::getEnd() const {
    return *endpoint;
}

Endpoint & MbsCompound::getEnd(size_t i) {
    return *endpoints[i];
}

const Endpoint & MbsCompound::getEnd(size_t i) const {
    return *endpoints[i];
}

const std::vector< Endpoint * > & MbsCompound::getEndpoints() const {
    return endpoints;
}

size_t MbsCompound::getNumberOfEndpoints() const {
    return endpoints.size();
}

void MbsCompound::doDirkin() {
    for (MbsObject * element : elements) {
        element->doDirkin();
    }
}

void MbsCompound::doPosition() {
    for (MbsObject * element : elements) {
        element->doPosition();
    }
}

void MbsCompound::doVelocity() {
    for (MbsObject * element : elements) {
        element->doVelocity();
    }
}

void MbsCompound::doAcceleration() {
    for (MbsObject * element : elements) {
        element->doAcceleration();
    }
}

void MbsCompound::doRneInward(bool withExternalForce) {
    for (std::vector< MbsObject * >::reverse_iterator it = elements.rbegin(); it != elements.rend(); it++) {
        (**it).doRneInward(withExternalForce);
    }
}

void MbsCompound::doAggregateBody() {
    for (std::vector< MbsObject * >::reverse_iterator it = elements.rbegin(); it != elements.rend(); it++) {
        (**it).doAggregateBody();
    }
}

TMatrixX MbsCompound::calculateMassMatrix() {
    // Store original values for velocity / acceleration / force of joints and base of robot.
    TVectorX dqOriginal = getJointVelocity();
    TVectorX ddqOriginal = getJointAcceleration();
    TVectorX tauOriginal = getJointForceTorque();
    CoordinateFrame cofbase = base->getCoordinateFrame();
    TVector3 vbase = cofbase.v;
    TVector3 omegabase = cofbase.omega;
    TVector3 dotvbase = cofbase.dotv;
    TVector3 dotomegabase = cofbase.dotomega;

    // Create mass matrix.
    int n = joints.size();
    TMatrixX M(n, n);

    // Initialize positions and velocities of mbs for calculation.
    cofbase.v.setZero();
    cofbase.dotv.setZero();
    cofbase.omega.setZero();
    cofbase.dotomega.setZero();
    TVectorX dq = TVectorX::Zero(n);
    setJointVelocity(dq);
    doPosition();
    doVelocity();

    // A (changing) vector for calculaiton of columns of M.
    TVectorX ddq = TVectorX::Zero(n);

    // Create M column-wise by repeated calculation of RNE.
    for (size_t i = 0; i < n; ++i) {
        ddq(i) = 1;
        setJointAcceleration(ddq);
        doAcceleration();
        doRneInward(false);
        M.col(i) = getJointForceTorque();
        ddq(i) = 0;
    }

    // Restore initial values for joints and base.
    setJointVelocity(dqOriginal);
    setJointAcceleration(ddqOriginal);
    setJointForceTorque(tauOriginal);
    cofbase.v = vbase;
    cofbase.omega = omegabase;
    cofbase.dotv = dotvbase;
    cofbase.dotomega = dotomegabase;

    // Return the calculated mass-matrix.
    return M;
}

TMatrixX MbsCompound::calculateMassMatrix2() {
    // Create mass matrix.
    int n = joints.size();
    int fb = freeBase ? 6 : 0;
    TMatrixX M(n + fb, n + fb);
    M.setZero(n + fb, n + fb);

    // Calculate direct kinematics
    //doDirkin();
    doPosition();

    // Calculate the composite bodies.
    doAggregateBody();

    for (size_t i = 0; i < joints.size(); ++i) {
        //std::cout << "i = " << i << "(" << joints[i]->getStateVectorPosition()<<")"<< " -- j = ";
        // Calculate resulting force for joint i, if only this joint is accelerated
        TVector3 n;
        TVector3 f;
        M(i + fb, i + fb) = joints[i]->calculateCRBAforce(f, n);
        if (i != 0) {
            //for(size_t j = 0; j < i; j++){
            //M(i+fb,j+fb) = M(j+fb,i+fb) = joints[j]->calculateCRBAforce(joints[i]->getCoordinateFrame().r,f,n);
            //}
            Joint1DOF * j = static_cast< mbslib::Joint1DOF * >(joints[i]->getPreceedingJoint());
            while (j != NULL) {
                //std::cout << " " << j->getJointId() << "(" << j->getStateVectorPosition()<<")";
                M(i + fb, j->getStateVectorPosition()) = M(j->getStateVectorPosition(), i + fb) = j->calculateCRBAforce(joints[i]->getCoordinateFrame().r, f, n);
                j = static_cast< mbslib::Joint1DOF * >(j->getPreceedingJoint());
            }
        }
        if (freeBase) {
            // tranform f and n to base cof and
            TVector3 fbase = base->getCoordinateFrame().R.transpose() * f;
            TVector3 nbase = base->getCoordinateFrame().R.transpose() * (n + (joints[i]->getCoordinateFrame().r - base->getCoordinateFrame().r).cross(f));
            // add set the according positions in M
            M.block< 3, 1 >(0, i + fb) = nbase;
            M.block< 3, 1 >(3, i + fb) = fbase;
            M.block< 1, 6 >(i + fb, 0) = M.block< 6, 1 >(0, i + fb);
        }
        //std::cout << std::endl;
    }
    if (freeBase) {
        // set upper left submatrix of M according to aggregated CoM, Inertia and Mass of whole system
        const AggregateBody & ab = base->getAggregateBody();
        M.block< 3, 3 >(0, 0) = base->getAggregateBody().I + ab.m * (ab.com.dot(ab.com) * TMatrix3x3::Identity() - ab.com * ab.com.transpose());
        M.block< 3, 3 >(3, 0) = -base->getAggregateBody().m * makeCrossproductMatrix(base->getAggregateBody().com);
        M.block< 3, 3 >(0, 3) = M.block< 3, 3 >(3, 0).transpose();
        M.block< 3, 3 >(3, 3) = TMatrix3x3::Identity() * base->getAggregateBody().m;
    }

    return M;
}

TMatrix6xX MbsCompound::calculateJacobian(const MbsObject & o, const TVector3 & relPos) {
    TMatrix6xX J;
    J.resize(6, dof);
    J.setZero();
    o.calculateJacobian(J, o.getCoordinateFrame().r + o.getCoordinateFrame().R * relPos);
    return J;
}

TVectorX MbsCompound::calculateCoriolisForceInJoints() {
    // Store original values for acceleration and force/torque of joints and base of robot.
    TVectorX ddqOriginal = getJointAcceleration();
    TVectorX tauOriginal = getJointForceTorque();
    CoordinateFrame & cofbase = base->getCoordinateFrame();
    TVector3 dotvbase = cofbase.dotv;
    TVector3 dotomegabase = cofbase.dotomega;

    // Create result vector.
    int n = joints.size();
    TVectorX C(n);

    // Initialize positions, velocities and acceleration of mbs for calculation.
    cofbase.dotv.setZero();
    cofbase.dotomega.setZero();
    TVectorX ddq = TVectorX::Zero(n);
    setJointAcceleration(ddq);
    doPosition();
    doVelocity();
    doAcceleration();

    doRneInward(false);
    C = getJointForceTorque();

    // Restore initial values for joints and base.
    setJointAcceleration(ddqOriginal);
    setJointForceTorque(tauOriginal);
    cofbase.dotv = dotvbase;
    cofbase.dotomega = dotomegabase;

    // Return the coriolis vector in joint space.
    return C;
}

TVectorX MbsCompound::calculateGravitationVectorInJoints() {
    return calculateGravitationVectorInJoints(gravitationVector);
}

TVectorX MbsCompound::calculateGravitationVectorInJoints(const TVector3 & g) {
    // Store original values for velocities, acceleration and forces/torques of joints and base of robot.
    TVectorX dqOriginal = getJointVelocity();
    TVectorX ddqOriginal = getJointAcceleration();
    TVectorX tauOriginal = getJointForceTorque();
    CoordinateFrame & cofbase = base->getCoordinateFrame();
    TVector3 vbase = cofbase.v;
    TVector3 omegabase = cofbase.omega;
    TVector3 dotvbase = cofbase.dotv;
    TVector3 dotomegabase = cofbase.dotomega;

    // Create result vector.
    int n = joints.size();
    TVectorX G(n);

    // Initialize positions, velocities and acceleration of mbs for calculation.
    cofbase.v.setZero();
    cofbase.omega.setZero();
    cofbase.dotv = -g;
    cofbase.dotomega.setZero();
    TVectorX dq = TVectorX::Zero(n);
    setJointVelocity(dq);
    TVectorX ddq = TVectorX::Zero(n);
    setJointAcceleration(ddq);
    doPosition();
    doVelocity();
    doAcceleration();

    doRneInward(false);
    G = getJointForceTorque();

    // Restore initial values for joints and base.
    setJointVelocity(dqOriginal);
    setJointAcceleration(ddqOriginal);
    setJointForceTorque(tauOriginal);
    cofbase.v = vbase;
    cofbase.omega = omegabase;
    cofbase.dotv = dotvbase;
    cofbase.dotomega = dotomegabase;

    // Return the gravitation vector in joint space.
    return G;
}

TVectorX MbsCompound::calculateExternalForceTorqueInJoints() {
    // Store original values for velocities and acceleration of joints and base of robot.
    TVectorX dqOriginal = getJointVelocity();
    TVectorX ddqOriginal = getJointAcceleration();
    TVectorX tauOriginal = getJointForceTorque();
    CoordinateFrame & cofbase = base->getCoordinateFrame();
    TVector3 vbase = cofbase.v;
    TVector3 omegabase = cofbase.omega;
    TVector3 dotvbase = cofbase.dotv;
    TVector3 dotomegabase = cofbase.dotomega;

    // Create result vector
    int n = joints.size();
    TVectorX tauext(n);

    // Initialize positions, velocities and acceleration of mbs for calculation.
    cofbase.v.setZero();
    cofbase.omega.setZero();
    cofbase.dotv.setZero();
    cofbase.dotomega.setZero();
    TVectorX dq = TVectorX::Zero(n);
    setJointVelocity(dq);
    TVectorX ddq = TVectorX::Zero(n);
    setJointAcceleration(ddq);
    doPosition();
    doVelocity();
    doAcceleration();
    doRneInward(true);
    tauext = getJointForceTorque();

    // Restore initial values for joints and base.
    setJointVelocity(dqOriginal);
    setJointAcceleration(ddqOriginal);
    setJointForceTorque(tauOriginal);
    cofbase.v = vbase;
    cofbase.omega = omegabase;
    cofbase.dotv = dotvbase;
    cofbase.dotomega = dotomegabase;

    // Return the resulting force/torque vector.
    return tauext;
}

///TODO: create superclass for MbsObject to handle integration and state-managemet in order to enable same for muscles etc.
void MbsCompound::integrate(TTime dt) {
    for (IIntegrate * it : objectsWithIntegrator) {
        it->integrate(dt);
    }
}

void MbsCompound::getControlLimits(TVectorX & lower, TVectorX & upper) const {
    if (!getNumberOfControlValues())
        return;
    lower.resize(getNumberOfControlValues());
    upper.resize(getNumberOfControlValues());

    int offset = 0;
    for (const Controllable * controllable : controllables) {
        for (size_t j = 0; j < controllable->getNumberOfControlValues(); ++j) {
            controllable->getControlLimits(lower(offset + j), upper(offset + j));
        }
        offset += controllable->getNumberOfControlValues();
    }
}
void MbsCompound::getStateLimits(TVectorX & lower, TVectorX & upper) const {
    if (lower.size() != getNumberOfStateVariables())
        lower.resize(getNumberOfStateVariables());
    if (upper.size() != getNumberOfStateVariables())
        upper.resize(getNumberOfStateVariables());

    size_t pos = 0;
    for (const IIntegrate * it : stateVectorJoints) {
        auto ll = (*it).getLowerStateLimits();
        auto ul = (*it).getUpperStateLimits();

        lower.segment(pos, (*it).getNumberOfStateVariables()) = ll;
        upper.segment(pos, (*it).getNumberOfStateVariables()) = ul;

        pos += (*it).getNumberOfStateVariables();
    }
    for (const IIntegrate * it : stateVectorNotJoints) {
        auto ll = (*it).getLowerStateLimits();
        auto ul = (*it).getUpperStateLimits();

        lower.segment(pos, (*it).getNumberOfStateVariables()) = ll;
        upper.segment(pos, (*it).getNumberOfStateVariables()) = ul;

        pos += (*it).getNumberOfStateVariables();
    }
}

bool MbsCompound::getIsStateVariableUnconstrainedAngle(unsigned int i) const {
    size_t pos = 0;
    for (const IIntegrate * it : stateVectorJoints) {
        int n = (*it).getNumberOfStateVariables();

        if (i >= pos && i < pos + n) {
            auto x = it;
            return x->getIsStateVariableUnconstrainedAngle(i - pos);
        }
        pos += n;
    }
    for (const IIntegrate * it : stateVectorNotJoints) {
        int n = (*it).getNumberOfStateVariables();

        if (i > pos && i < pos + n) {
            auto x = it;
            return x->getIsStateVariableUnconstrainedAngle(i - pos);
        }
        pos += n;
    }
    return false;
}

std::string MbsCompound::getControlValueName(unsigned int i) const {
    int offset = 0;
    int n;
    for (int i = 0; i < (n = getNumberOfControllables()); i++) {
        const Controllable & controllable = getControllable(i);

        int ii = i - offset;
        if (i >= offset && i < offset + n) {
            return controllable.getControlValueName(ii);
        }
        offset += n;
    }
    return "";
}

std::string MbsCompound::getStateVariableName(unsigned int i) const {
    size_t pos = 0;
    for (std::vector< IIntegrate * >::const_iterator it = stateVectorJoints.begin(); it != stateVectorJoints.end(); it++) {
        int n = (**it).getNumberOfStateVariables();

        if (i >= pos && i < pos + n) {
            int ii = i - pos;
            auto x = *it;
            return x->getStateVariableName(ii);
        }
        pos += n;
    }
    for (std::vector< IIntegrate * >::const_iterator it = stateVectorNotJoints.begin(); it != stateVectorNotJoints.end(); it++) {
        int n = (**it).getNumberOfStateVariables();

        if (i >= pos && i < pos + n) {
            auto x = *it;
            return x->getStateVariableName(i - pos);
        }
        pos += n;
    }
    std::stringstream ss;
    ss << "compound_" << i;
    return ss.str();
}

void MbsCompound::storeState() {
    for (IIntegrate * it : objectsWithIntegrator) {
        it->storeState();
    }
}

void MbsCompound::restoreState() {
    for (IIntegrate * it : objectsWithIntegrator) {
        it->restoreState();
    }
}

size_t MbsCompound::getNumberOfStateVariables() const {
    return statesOfJoints + statesOfNonJoints;
}

TVectorX MbsCompound::getState() const {
    TVectorX s(getNumberOfStateVariables());
    getState(s);
    return s;
}

void MbsCompound::getState(TVectorX & s) const {
    if (s.size() != getNumberOfStateVariables())
        s.resize(getNumberOfStateVariables());

    size_t pos = 0;
    for (std::vector< IIntegrate * >::const_iterator it = stateVectorJoints.begin(); it != stateVectorJoints.end(); it++) {
        s.segment(pos, (**it).getNumberOfStateVariables()) = (**it).getState();
        pos += (**it).getNumberOfStateVariables();
    }
    for (std::vector< IIntegrate * >::const_iterator it = stateVectorNotJoints.begin(); it != stateVectorNotJoints.end(); it++) {
        s.segment(pos, (**it).getNumberOfStateVariables()) = (**it).getState();
        pos += (**it).getNumberOfStateVariables();
    }
}

TVectorX MbsCompound::getDStateDt() const {
    TVectorX s(getNumberOfStateVariables());
    size_t pos = 0;
    for (std::vector< IIntegrate * >::const_iterator it = stateVectorJoints.begin(); it != stateVectorJoints.end(); it++) {
        s.segment(pos, (**it).getNumberOfStateVariables()) = (**it).getDStateDt();
        pos += (**it).getNumberOfStateVariables();
    }
    for (std::vector< IIntegrate * >::const_iterator it = stateVectorNotJoints.begin(); it != stateVectorNotJoints.end(); it++) {
        s.segment(pos, (**it).getNumberOfStateVariables()) = (**it).getDStateDt();
        pos += (**it).getNumberOfStateVariables();
    }
    return s;
}

void MbsCompound::setState(const TVectorX & s) {
    int size = s.size();
    size_t pos = 0;
    for (std::vector< IIntegrate * >::iterator it = stateVectorJoints.begin(); it != stateVectorJoints.end(); it++) {
        (**it).setState(s.segment(pos, (**it).getNumberOfStateVariables()));
        pos += (**it).getNumberOfStateVariables();
    }
    for (std::vector< IIntegrate * >::iterator it = stateVectorNotJoints.begin(); it != stateVectorNotJoints.end(); it++) {
        (**it).setState(s.segment(pos, (**it).getNumberOfStateVariables()));
        pos += (**it).getNumberOfStateVariables();
    }
}

size_t MbsCompound::getNumberOfJoints() const {
    return joints.size();
}

std::vector< Joint1DOF * > & MbsCompound::getJoints() {
    return joints;
}

const std::vector< Joint1DOF * > & MbsCompound::getJoints() const {
    return joints;
}

int MbsCompound::getDOF() const {
    return dof;
}

void MbsCompound::setJointPosition(const TVectorX & q) {
    for (size_t i = 0; i < joints.size(); i++) {
        joints[i]->getJointState().q = q(i);
    }
}

void MbsCompound::setJointVelocity(const TVectorX & dq) {
    for (size_t i = 0; i < joints.size(); i++) {
        joints[i]->getJointState().dotq = dq(i);
    }
}

void MbsCompound::setJointAcceleration(const TVectorX & ddq) {
    for (size_t i = 0; i < joints.size(); i++) {
        joints[i]->getJointState().ddotq = ddq(i);
    }
}

void MbsCompound::setJointForceTorque(const TVectorX & tau) {
    for (size_t i = 0; i < joints.size(); i++) {
        joints[i]->setJointForceTorque(tau(i));
    }
}

TVectorX MbsCompound::getJointPosition() {
    TVectorX q(joints.size());
    for (size_t i = 0; i < joints.size(); i++) {
        q(i) = joints[i]->getJointState().q;
    }
    return q;
}

TVectorX MbsCompound::getJointVelocity() {
    TVectorX dq(joints.size());
    for (size_t i = 0; i < joints.size(); i++) {
        dq(i) = joints[i]->getJointState().dotq;
    }
    return dq;
}

TVectorX MbsCompound::getJointAcceleration() {
    TVectorX ddq(joints.size());
    for (size_t i = 0; i < joints.size(); i++) {
        ddq(i) = joints[i]->getJointState().ddotq;
    }
    return ddq;
}

TVectorX MbsCompound::getJointForceTorque() {
    TVectorX tau(joints.size());
    for (size_t i = 0; i < joints.size(); i++) {
        tau(i) = joints[i]->getJointForceTorque();
    }
    return tau;
}

const TVector3 & MbsCompound::getGravitation() const {
    return gravitationVector;
}

void MbsCompound::setGravitation(const TVector3 & g) {
    gravitationVector = g;
}

size_t MbsCompound::getNumberOfActiveDrives() const {
    return activeDrives.size();
}

void MbsCompound::setActiveDrivePosition(const TVectorX & q) {
    for (size_t i = 0; i < activeDrives.size(); i++) {
        activeDrives[i]->setDesiredPosition(q(i));
    }
}

void MbsCompound::setActiveDriveVelocity(const TVectorX & dq) {
    for (size_t i = 0; i < activeDrives.size(); i++) {
        activeDrives[i]->setDesiredVelocity(dq(i));
    }
}

void MbsCompound::doForwardDrives() {
    for (size_t i = 0; i < drives.size(); i++) {
        drives[i]->doForwardDrive();
    }
}

void MbsCompound::doInverseDrives() {
    for (size_t i = 0; i < drives.size(); i++) {
        drives[i]->doInverseDrive();
    }
}

size_t MbsCompound::getNumberOfControllables() const {
    return controllables.size();
}

const Controllable & MbsCompound::getControllable(size_t i) const {
    return *controllables[i];
}

unsigned int MbsCompound::getNumberOfControlValues() const {
    return numberOfControlValues;
}

void MbsCompound::setControlValues(const TVectorX & u) { /*
  assert(u.size() == getNumberOfControlValues());
  for(size_t i = 0; i < getNumberOfControlValues(); i++){
    //for the time being we assume that each controllable has exaclty one control value
    ///TODO: change this!
    assert(controllables[i]->getNumberOfControlValues() == 1);
    controllables[i]->setControlValue(u(i));
  }
  */
    unsigned int upos = 0;
    for (size_t i = 0; i < controllables.size(); i++) {
        for (unsigned int j = 0; j < controllables[i]->getNumberOfControlValues(); j++) {
            controllables[i]->setControlValue(u(upos), j);
            upos++;
        }
    }
}

TVectorX MbsCompound::getControlValues() const {
    TVectorX u(getNumberOfControlValues());
    /*
  for(size_t i = 0; i < getNumberOfControlValues(); i++){
    //for the time being we assume that each controllable has exaclty one control value
    ///TODO: change this!
    assert(controllables[i]->getNumberOfControlValues() == 1);
    u(i) = controllables[i]->getControlValue();
  }
  */
    unsigned int upos = 0;
    for (size_t i = 0; i < controllables.size(); i++) {
        for (unsigned int j = 0; j < controllables[i]->getNumberOfControlValues(); j++) {
            u(upos) = controllables[i]->getControlValue(j);
            upos++;
        }
    }
    return u;
}

TMatrixX MbsCompound::calculateDtauDcontrol() {
    TMatrixX m(getDOF(), getNumberOfControlValues());

    //reset all forces introduced to the system
    forceGeneratorSet.resetForces();
    //store joint torque, acceleration and velocity
    TVectorX tau = getJointForceTorque();
    TVectorX ddq = getJointAcceleration();
    TVectorX dq = getJointVelocity();
    //and then clear them
    setJointAcceleration(TVectorX::Zero(getDOF()));
    setJointVelocity(TVectorX::Zero(getDOF()));

    //calculate direct kinematics
    doDirkin();

    //iterate over all Controllables and let each time exaclty one of them
    //introduce the derivative of it's force w.r.t. the control value into the system
    //as the mbs will incorporate these forces in a linear way, this will lead
    //to derivatives of the following calculations w.r.t. the control value

    unsigned int upos = 0;
    for (size_t i = 0; i < controllables.size(); i++) {
        //for the time being we assume that each controllable has exaclty one control value
        ///TODO: change this!
        //assert(controllables[i]->getNumberOfControlValues() == 1);

        for (unsigned int j = 0; j < controllables[i]->getNumberOfControlValues(); j++) {
            controllables[i]->setDeriveMode(true, j);
            controllables[i]->applyForce();

            //calculate inward sweep of rne to get joint torques/forces required to
            //hold against the forces introduced by the ith Controllable
            doRneInward(true);
            m.col(upos) = -getJointForceTorque();

            controllables[i]->resetForce();
            controllables[i]->setDeriveMode(false, j);

            upos++;
        }
    }

    //restore joint torque, acceleration and velocity
    setJointForceTorque(tau);
    setJointAcceleration(ddq);
    setJointVelocity(dq);
    return m;
}

TMatrixX MbsCompound::calculateDddqDcontrol() {
    return calculateMassMatrix2().inverse() * calculateDtauDcontrol();
}

TMatrixX MbsCompound::calculateDddqDcontrol2() {
    TMatrixX m(getDOF(), getNumberOfControlValues());

    //reset all forces introduced to the system
    forceGeneratorSet.resetForces();
    //store joint torque, acceleration and velocity
    TVectorX tau = getJointForceTorque();
    TVectorX ddq = getJointAcceleration();
    TVectorX dq = getJointVelocity();
    //and then clear them
    setJointForceTorque(TVectorX::Zero(getDOF()));
    setJointVelocity(TVectorX::Zero(getDOF()));

    unsigned int upos = 0;

    for (size_t i = 0; i < controllables.size(); i++) {
        for (unsigned int j = 0; j < controllables[i]->getNumberOfControlValues(); j++) {
            //for the time being we assume that each controllable has exaclty one control value
            ///TODO: change this!
            //assert(controllables[i]->getNumberOfControlValues() == 1);

            controllables[i]->setDeriveMode(true, j);
            controllables[i]->applyForce();

            doABA(TVector3(0, 0, 0), true);
            m.col(upos) = getJointAcceleration();

            controllables[i]->resetForce();
            controllables[i]->setDeriveMode(false, j);

            upos++;
        }
    }

    //restore joint torque, acceleration and velocity
    setJointForceTorque(tau);
    setJointAcceleration(ddq);
    setJointVelocity(dq);

    return m;
}

const std::vector< MbsObject * > & MbsCompound::getElements() const {
    return elements;
}

void MbsCompound::addUserStateVariable(UserStateVariable & variable) {
    userStateVariables.push_back(&variable);
    stateVectorNotJoints.push_back(&variable);
    statesOfNonJoints += variable.getNumberOfStateVariables();
}

void MbsCompound::addElement(MbsObject * element) {
    elements.push_back(element);
    IIntegrate * iint = dynamic_cast< IIntegrate * >(element);
    if (iint) {
        objectsWithIntegrator.push_back(iint);
        Joint1DOF * j1d = dynamic_cast< Joint1DOF * >(iint);

        if (j1d) {
            stateVectorJoints.push_back(j1d);
            statesOfJoints += j1d->getNumberOfStateVariables();
        } else {
            stateVectorNotJoints.push_back(iint);
            statesOfNonJoints += iint->getNumberOfStateVariables();
        }
    };

    addParametrizedObject(*element);

    Joint1DOF * joint = dynamic_cast< Joint1DOF * >(element);
    if (joint) {
        joints.push_back(joint);
    }

    Base * b = dynamic_cast< Base * >(element);
    if (b) {
        assert(base == NULL);
        base = b;
        FreeBase * fb = dynamic_cast< FreeBase * >(b);
        if (fb) {
            freeBase = true;
        }
    }

    Endpoint * e = dynamic_cast< Endpoint * >(element);
    if (e) {
        endpoints.push_back(e);
        endpoint = e;
    }

    dof += element->getDOF();
}

void MbsCompound::addDrive(Drive * drive) {
    drives.push_back(drive);
    ActiveDrive * ad = dynamic_cast< ActiveDrive * >(drive);
    if (ad) {
        activeDrives.push_back(ad);
    }

    ParametrizedObject * po = dynamic_cast< ParametrizedObject * >(drive);
    if (po) {
        addParametrizedObject(*po);
    }
}

void MbsCompound::addForceGenerator(ForceGenerator * fg) {
    forceGeneratorSet.add(*fg);

    ParametrizedObject * po = dynamic_cast< ParametrizedObject * >(fg);
    if (po) {
        addParametrizedObject(*po);
    }

    Controllable * c = dynamic_cast< Controllable * >(fg);
    if (c) {
        controllables.push_back(c);
        numberOfControlValues += c->getNumberOfControlValues();
    }

    IIntegrate * iint = fg->getIntegrator(); // dynamic_cast<IIntegrate*> (fg);
    if (iint) {
        objectsWithIntegrator.push_back(iint);
        stateVectorNotJoints.push_back(iint);
        statesOfNonJoints += iint->getNumberOfStateVariables();
    };
}

MbsObject * MbsCompound::getElementByName(const std::string & name) const {
    MbsObject * retval = nullptr;
    for (MbsObject * element : elements) {
        if (element != nullptr && element->getName() == name) {
            retval = element;
            break;
        }
    }
    return retval;
}
Endpoint * MbsCompound::getEndpointByName(const std::string & name) const {
    Endpoint * retval = nullptr;
    for (Endpoint * ep : endpoints) {
        if (ep != nullptr && ep->getName() == name) {
            retval = ep;
            break;
        }
    }
    return retval;
}
Joint1DOF * MbsCompound::getJointByName(const std::string & name) const {
    Joint1DOF * retval = nullptr;
    for (Joint1DOF * joint : joints) {
        if (joint != nullptr && joint->getName() == name) {
            retval = joint;
            break;
        }
    }
    return retval;
}

const ForceGeneratorSet & MbsCompound::getForceGeneratorSet() const {
    return forceGeneratorSet;
}
const std::vector< ActiveDrive * > & MbsCompound::getActiveDrives() const {
    return activeDrives;
}
