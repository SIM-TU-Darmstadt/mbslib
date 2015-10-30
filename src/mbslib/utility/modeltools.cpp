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
 * \file mbslib/utility/modeltools.cpp
 *
 */
#include <mbslib/utility/modeltools.hpp>
using namespace mbslib;
TMatrix3x3 mbslib::makeInertiaTensor(TScalar xx, TScalar yy, TScalar zz, TScalar xy, TScalar xz, TScalar yz) {
    TMatrix3x3 I;
    I(0, 0) = xx;
    I(1, 1) = yy;
    I(2, 2) = zz;

    I(0, 1) = I(1, 0) = xy;
    I(0, 2) = I(2, 0) = xz;
    I(1, 2) = I(2, 1) = yz;

    return I;
}

TMatrix3x3 mbslib::makeInertiaTensorCylinderX(TScalar radius, TScalar length, TScalar mass) {
    TScalar Ixx = 0.5 * mass * radius * radius;
    TScalar Iyy = mass * radius * radius / 4.f + mass * length * length / 12.f;
    TScalar Izz = Iyy;

    return makeInertiaTensor(Ixx, Iyy, Izz);
}

TMatrix3x3 mbslib::makeInertiaTensorCylinderY(TScalar radius, TScalar length, TScalar mass) {
    TScalar Ixx = mass * radius * radius / 4.f + mass * length * length / 12.f;
    TScalar Iyy = 0.5 * mass * radius * radius;
    TScalar Izz = Ixx;

    return makeInertiaTensor(Ixx, Iyy, Izz);
}

TMatrix3x3 mbslib::makeInertiaTensorCylinderZ(TScalar radius, TScalar length, TScalar mass) {
    TScalar Ixx = mass * radius * radius / 4.f + mass * length * length / 12.f;
    TScalar Iyy = Ixx;
    TScalar Izz = 0.5 * mass * radius * radius;

    return makeInertiaTensor(Ixx, Iyy, Izz);
}
