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
 * \file mbslib/elements/spring/model/LinearSpringWithRopeModel.cpp
 * Definition of mbslib ::LinearSpringWithRopeModel
 */

#include <mbslib/elements/spring/model/LinearSpringWithRopeModel.hpp>

using namespace mbslib;

LinearSpringWithRopeModel::LinearSpringWithRopeModel(TScalar k, TScalar d, TScalar ropeLength)
    : k(k)
    , d(d)
    , ropeLength(ropeLength) {
}

TScalar LinearSpringWithRopeModel::calculateForce(TScalar l, TScalar dl) const {
    //return (l > ropeLength)?( k * ( l - ropeLength ) + d * dl ):TScalar(0);
    TScalar f;
    condassign(f, (l - ropeLength), (k * (l - ropeLength) + d * dl), TScalar(0));
    return f;
}

TScalar LinearSpringWithRopeModel::calculateDForceDLength(TScalar l, TScalar dl) const {
    TScalar f;
    condassign(f, (l - ropeLength), (k), TScalar(0));
    return f;
}

TScalar LinearSpringWithRopeModel::calculateDForceDVelocity(TScalar l, TScalar dl) const {
    TScalar f;
    condassign(f, (l - ropeLength), (dl), TScalar(0));
    return f;
}

SpringModel * LinearSpringWithRopeModel::clone() const {
    return new LinearSpringWithRopeModel(k, d, ropeLength);
}

LinearSpringWithRopeModel::~LinearSpringWithRopeModel() {
}
