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
 * \file mbslib/elements/spring/model/LinearSpringModel.cpp
 * Definition of mbslib ::LinearSpringModel
 */

#include <mbslib/elements/spring/model/LinearSpringModel.hpp>

using namespace mbslib;

LinearSpringModel::LinearSpringModel(TScalar k, TScalar d)
    : k(k)
    , d(d) {
}

TScalar LinearSpringModel::calculateForce(TScalar l, TScalar dl) const {
    return k * l + d * dl;
}

TScalar LinearSpringModel::calculateDForceDLength(TScalar l, TScalar dl) const {
    return k;
}

TScalar LinearSpringModel::calculateDForceDVelocity(TScalar l, TScalar dl) const {
    return d;
}
SpringModel * LinearSpringModel::clone() const {
    return new LinearSpringModel(k, d);
}
LinearSpringModel::~LinearSpringModel() {
}
