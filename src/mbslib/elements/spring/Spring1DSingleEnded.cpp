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
 * \file mbslib/elements/spring/Spring1DSingleEnded.cpp
 * Definition of mbslib ::Spring1DSingleEnded
 */

#include <mbslib/elements/spring/Spring1DSingleEnded.hpp>

using namespace mbslib;

Spring1DSingleEnded::Spring1DSingleEnded(const SpringModel & model, Joint1DOF & j1, const std::string & name)
    : Spring(model, name)
    , j1(j1) {
}

void Spring1DSingleEnded::applyForce() {
    TScalar deltaQ = j1.getJointState().q;
    TScalar deltaDQ = j1.getJointState().dotq;

    f = model->calculateForce(deltaQ, deltaDQ);

    j1.setExternalJointForceTorque(j1.getExternalJointForceTorque() - f);
}

/**
 * Reset forces of points attached to joints.
 */
void Spring1DSingleEnded::resetForce() {
    j1.setExternalJointForceTorque(0);
}
Spring1DSingleEnded::~Spring1DSingleEnded() {
}
