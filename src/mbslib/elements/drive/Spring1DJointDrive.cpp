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
 * \file mbslib/elements/drive/Spring1DJointDrive.cpp
 * Definition of mbslib ::Spring1DJointDrive
 */
#include <mbslib/elements/drive/Spring1DJointDrive.hpp>

using namespace mbslib;

Spring1DJointDrive::Spring1DJointDrive(const SpringModel & model, Joint1DOF & j1, const std::string & name)
    : Spring(model, name)
    , j1(j1)
    , position(0)
    , velocity(0)
    , deriveMode(false) {
}

void Spring1DJointDrive::applyForce() {
    TScalar deltaQ = j1.getJointState().q - position;
    TScalar deltaDQ = j1.getJointState().dotq - velocity;

    if (!deriveMode) {
        f = model->calculateForce(deltaQ, deltaDQ);
    } else {
        switch (deriveBy) {
        case 0:
            f = model->calculateDForceDLength(deltaQ, deltaDQ);
            break;
        case 1:
            f = model->calculateDForceDVelocity(deltaQ, deltaDQ);
            break;
        default:
            assert(0);
            f = 0;
        }
    }

    j1.setExternalJointForceTorque(j1.getExternalJointForceTorque() - f);
}

/**
 * Reset forces of points attached to joints.
 */
void Spring1DJointDrive::resetForce() {
    j1.setExternalJointForceTorque(0);
}

void Spring1DJointDrive::setDeriveMode(bool dm, unsigned int valueId) {
    ///TODO: we need a way to get derivatives of a spring by length and dotlength
    deriveMode = dm;
    deriveBy = valueId;
}

bool Spring1DJointDrive::getDeriveMode() const {
    return deriveMode;
}

void Spring1DJointDrive::setControlValue(TScalar c, unsigned int valueId) {
    if (valueId == 0) {
        position = c;
        return;
    }
    if (valueId == 1) {
        velocity = c;
        return;
    }
    assert(0);
}

TScalar Spring1DJointDrive::getControlValue(unsigned int valueId) const {
    if (valueId == 0) {
        return position;
    }
    if (valueId == 1) {
        return velocity;
    }
    assert(0);
    return 0;
}

const std::string & Spring1DJointDrive::getControlValueName(unsigned int valueId) const {
    static std::string pos = "position";
    static std::string vel = "velocity";
    static std::string unknown = "unknown control value";
    if (valueId == 0) {
        return pos;
    }
    if (valueId == 1) {
        return vel;
    }
    return unknown;
    assert(0);
}

unsigned int Spring1DJointDrive::getNumberOfControlValues(void) const {
    return 2;
}

Spring1DJointDrive::~Spring1DJointDrive() {
}
