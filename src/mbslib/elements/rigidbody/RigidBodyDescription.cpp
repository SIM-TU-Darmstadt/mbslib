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
 * \file mbslib/elements/rigidbody/RigidBodyDescription.cpp
 * Definition of mbslib::RigidBodyDescription
 */

#include <mbslib/elements/rigidbody/RigidBodyDescription.hpp>

using namespace mbslib;

RigidBodyDescription::RigidBodyDescription() {
    r.setZero();
    m = 0;
    com.setZero();
    I.setZero();
}

RigidBodyDescription::RigidBodyDescription(const TVector3 & r, const TVector3 & com, TScalar m, const TMatrix3x3 & I)
    : r(r)
    , com(com)
    , m(m)
    , I(I) {
}

RigidBodyDescription RigidBodyDescription::mirror(const TVector3 & normal) {
    return RigidBodyDescription(
        r - 2 * normal * normal.dot(r),
        com - 2 * normal * normal.dot(com),
        m,
        I);
}

RigidBodyDescription::~RigidBodyDescription() {
}
