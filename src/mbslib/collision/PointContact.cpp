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
 * \file mbslib/collision/PointContact.cpp
 * Definition of mbslib::PointContact
 */
#include <mbslib/collision/PointContact.hpp>

using namespace mbslib;

PointContact::PointContact(Endpoint & endpoint, const TVector3 & normal, TScalar depth)
    : endpoint(&endpoint)
    , normal(normal)
    , depth(depth) {
}

PointContact::~PointContact() {
}

Endpoint & PointContact::getEndpoint() {
    return *endpoint;
}

const Endpoint & PointContact::getEndpoint() const {
    return *endpoint;
}

TScalar PointContact::getDepth() const {
    return depth;
}

TScalar PointContact::getPenetrationVelocity() const {
    return -getVelocityLocalCoordinateSystem().dot(getNormalLocalCoordinateSystem());
}

const TVector3 & PointContact::getNormalLocalCoordinateSystem() const {
    return normal;
}

const TVector3 PointContact::getNormalWorldCoordinateSystem() const {
    return endpoint->getCoordinateFrame().R * normal;
}

const TVector3 PointContact::getVelocityLocalCoordinateSystem() const {
    return endpoint->getCoordinateFrame().v;
}

const TVector3 PointContact::getVelocityWorldCoordinateSystem() const {
    return endpoint->getCoordinateFrame().R * endpoint->getCoordinateFrame().v;
}
