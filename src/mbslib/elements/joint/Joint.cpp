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
 * \file mbslib/elements/joint/Joint.cpp
 * Definition of mbslib::Joint
 */

#include <mbslib/elements/joint/Joint.hpp>

using namespace mbslib;

Joint::Joint(MbsObject & pred, int jointId, int stateVectorPosition, const std::string & name)
    : MbsObjectOneSucc(pred, name)
    , jointId(jointId)
    , predJoint(NULL)
    , stateVectorPosition(stateVectorPosition) {
    MbsObject * p = &pred;
    //std::cout << "adding joint " << jointId << "(" << stateVectorPosition << ") ";
    while (p != NULL) {
        predJoint = dynamic_cast< Joint * >(p);
        if (predJoint) {
            break;
        }
        p = p->getPredecessor();
    }
}

void Joint::doABASweep1hyb(const std::vector< bool > & doDirectDyn) {
    if (doDirectDyn[jointId]) {
        doABASweep1fwd();
    } else {
        doABASweep1inv();
    }
}

void Joint::doABASweep2hyb(const std::vector< bool > & doDirectDyn) {
    if (doDirectDyn[jointId]) {
        doABASweep2fwd();
    } else {
        doABASweep2inv();
    }
}

void Joint::doABASweep3hyb(const std::vector< bool > & doDirectDyn) {
    if (doDirectDyn[jointId]) {
        doABASweep3fwd();
    } else {
        doABASweep3inv();
    }
}

int Joint::getJointId() const {
    return jointId;
}

int Joint::getStateVectorPosition() const {
    return stateVectorPosition;
}

Joint * Joint::getPreceedingJoint() {
    return predJoint;
}

Joint::~Joint() {
}
