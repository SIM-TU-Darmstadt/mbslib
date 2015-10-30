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
 * \file mbslib/elements/object/MbsObjectTwoSucc.cpp
 * Definition of mbslib ::MbsObjectTwoSucc
 */

#include <mbslib/elements/object/MbsObjectTwoSucc.hpp>

using namespace mbslib;

bool MbsObjectTwoSucc::isValid() const {
    return (MbsObject::isValid() && (succOne != NULL) && (succTwo != NULL));
}

MbsObjectTwoSucc::MbsObjectTwoSucc(const std::string & n)
    : MbsObject(n)
    , succOne(NULL)
    , succTwo(NULL) {
}

MbsObjectTwoSucc::MbsObjectTwoSucc(MbsObject & p, const std::string & n)
    : MbsObject(p, n)
    , succOne(NULL)
    , succTwo(NULL) {
}

bool MbsObjectTwoSucc::addSuccessor(MbsObject & s) {
    if (!succOne) {
        succOne = &s;
        return true;
    }
    if (!succTwo) {
        succTwo = &s;
        return true;
    }
    setupError = true;
    return false;
}

const MbsObject * MbsObjectTwoSucc::getSuccessor(size_t index) const {
    if (index == 0)
        return succOne;
    if (index == 1)
        return succTwo;
    return 0;
}
size_t MbsObjectTwoSucc::getSuccessorCount() const {
    return 2;
}
MbsObjectTwoSucc::~MbsObjectTwoSucc() {
}
