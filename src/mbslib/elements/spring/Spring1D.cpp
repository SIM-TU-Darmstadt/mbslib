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
 * \file mbslib/elements/spring/Spring1D.cpp
 * Definition of mbslib ::Spring1D
 */

#include <mbslib/elements/spring/Spring1D.hpp>

using namespace mbslib;

Spring1D::Spring1D(const SpringModel & model, Joint1DOF & j1, Joint1DOF & j2, const std::string & name)
    : Spring(model, name)
    , j1(j1)
    , j2(j2) {
}

void Spring1D::applyForce() {
    TScalar deltaQ = j2.getJointState().q - j1.getJointState().q;
    TScalar deltaDQ = j2.getJointState().dotq - j1.getJointState().dotq;

    f = model->calculateForce(deltaQ, deltaDQ);

    j1.setExternalJointForceTorque(j1.getExternalJointForceTorque() + f);
    j2.setExternalJointForceTorque(j2.getExternalJointForceTorque() - f);
}

/**
 * Reset forces of points attached to joints.
 */
void Spring1D::resetForce() {
    j1.setExternalJointForceTorque(0);
    j2.setExternalJointForceTorque(0);
}
Spring1D::~Spring1D() {
}
