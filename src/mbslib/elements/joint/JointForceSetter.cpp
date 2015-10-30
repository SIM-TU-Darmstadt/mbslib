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
 * \file mbslib/elements/joint/JointForceSetter.cpp
 * Definition of mbslib ::JointForceSetter
 */

#include <mbslib/elements/joint/JointForceSetter.hpp>

using namespace mbslib;

JointForceSetter::JointForceSetter(Joint1DOF & j1)
    : j1(j1)
    , deriveMode(false)
    , tau(0) {
    name = std::string("JointForceSetter at ");
    name.append(j1.getName());
}

JointForceSetter::JointForceSetter(Joint1DOF & j1, const std::string & name)
    : j1(j1)
    , deriveMode(false)
    , tau(0)
    , name(name) {
}

void JointForceSetter::applyForce() {
    TScalar f;

    if (!deriveMode) {
        f = tau;
    } else {
        f = 1;
    }

    j1.setExternalJointForceTorque(j1.getExternalJointForceTorque() + f);
}

void JointForceSetter::resetForce() {
    j1.setExternalJointForceTorque(0);
}

void JointForceSetter::setDeriveMode(bool dm, unsigned int valueId) {
    deriveMode = dm;
    assert(valueId == 0);
}

bool JointForceSetter::getDeriveMode() const {
    return deriveMode;
}

void JointForceSetter::setControlValue(TScalar c, unsigned int valueId) {
    if (valueId == 0) {
        tau = c;
        return;
    }
    assert(0);
}

TScalar JointForceSetter::getControlValue(unsigned int valueId) const {
    if (valueId == 0) {
        return tau;
    }
    assert(0);
    return 0;
}

const std::string & JointForceSetter::getControlValueName(unsigned int valueId) const {
    static std::string tau = "joint force/torqe";
    static std::string unknown = "unknown control value";
    if (valueId == 0) {
        return tau;
    }
    assert(0);
    return unknown;
}

unsigned int JointForceSetter::getNumberOfControlValues(void) const {
    return 1;
}

const std::string & JointForceSetter::getName() const {
    return name;
}

IIntegrate * JointForceSetter::getIntegrator() {
    return NULL;
}

JointForceSetter::~JointForceSetter() {
}
