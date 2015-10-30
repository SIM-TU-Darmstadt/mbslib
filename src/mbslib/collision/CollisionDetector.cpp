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
 * \file mbslib/collision/CollisionDetector.cpp
 * Definition of mbslib ::CollisionDetector
 */
#include <mbslib/collision/CollisionDetector.hpp>

using namespace mbslib;

CollisionDetector::CollisionDetector()
    : p0(TVector3(0, 0, 0))
    , n0(TVector3(0, 0, 1))
    , name("CollisionDetector") {
}

CollisionDetector::CollisionDetector(const TVector3 & p0, const TVector3 & n0)
    : p0(p0)
    , n0(n0)
    , name("CollisionDetector") {
}

void CollisionDetector::addEndpoint(Endpoint & e) {
    endpoints.push_back(std::pair< Endpoint *, PointContactHandler * >(&e, static_cast< PointContactHandler * >(NULL)));
}

void CollisionDetector::addEndpoint(Endpoint & e, PointContactHandler & ch) {
    endpoints.push_back(std::pair< Endpoint *, PointContactHandler * >(&e, &ch));
}

void CollisionDetector::detectContacts() {
    contacts.clear();
    contactHandlers.clear();

    for (std::vector< std::pair< Endpoint *, PointContactHandler * > >::iterator it = endpoints.begin(); it != endpoints.end(); it++) {
        //calculate penetration depth
        TScalar depth = -(it->first->getCoordinateFrame().r - p0).dot(n0);

        //if the point is below the plane, we have to check, whether the velocity is not separating
        if (depth >= 0) {
            // calculate collision normal in endpoint-coordinates (we require this value twice, so we pre-calculate it)
            TVector3 n = it->first->getCoordinateFrame().R.transpose() * n0;
            // check, if the velocity of the contact point goes in colliding direction (and not separating)
            if ((it->first->getCoordinateFrame().v.dot(n)) < 0) {
                PointContact contact(*(it->first), n, depth);
                contacts.push_back(contact);
                if (it->second) {
                    contactHandlers.push_back(std::pair< PointContact, PointContactHandler * >(contact, it->second));
                }
            }
        }
    }
}

void CollisionDetector::applyForce() {
    for (std::vector< std::pair< PointContact, PointContactHandler * > >::iterator it = contactHandlers.begin(); it != contactHandlers.end(); it++) {
        it->second->applyForce(it->first);
        appliedForces.push_back(*it);
    }
}

void CollisionDetector::resetForce() {
    for (std::vector< std::pair< PointContact, PointContactHandler * > >::iterator it = appliedForces.begin(); it != appliedForces.end(); it++) {
        it->second->resetForce(it->first);
    }
    appliedForces.clear();
}

const std::vector< PointContact > & CollisionDetector::getContacts() const {
    return contacts;
}

const std::string & CollisionDetector::getName() const {
    return name;
}

IIntegrate * CollisionDetector::getIntegrator() {
    return 0;
}

CollisionDetector::~CollisionDetector() {
}
