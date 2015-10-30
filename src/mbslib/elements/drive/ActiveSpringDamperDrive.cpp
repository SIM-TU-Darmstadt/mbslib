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
 * \file mbslib/elements/drive/ActiveSpringDamperDrive.cpp
 * Definition of mbslib ::ActiveSpringDamperDrive
 */
#include <mbslib/elements/drive/ActiveSpringDamperDrive.hpp>

using namespace mbslib;

ActiveSpringDamperDrive::ActiveSpringDamperDrive(Joint1DOF & joint, TScalar springCoeff_, TScalar damperCoeff_, const std::string & name)
    : ActiveDrive(joint, name)
    , springCoeff(springCoeff_)
    , damperCoeff(damperCoeff_) {
}

void ActiveSpringDamperDrive::doForwardDrive() {
    TScalar springEffort;
    springEffort = -springCoeff * (attachedJoint->getJointState().q - q_des) - damperCoeff * (attachedJoint->getJointState().dotq - dq_des);
    attachedJoint->setJointForceTorque(springEffort);
}
void ActiveSpringDamperDrive::doInverseDrive() {
    TScalar springEffort;
    springEffort = -springCoeff * (attachedJoint->getJointState().q - q_des) - damperCoeff * (attachedJoint->getJointState().dotq - dq_des);
    attachedJoint->setJointForceTorque(attachedJoint->getJointForceTorque() - springEffort);
}

ActiveSpringDamperDrive::~ActiveSpringDamperDrive() {
}
