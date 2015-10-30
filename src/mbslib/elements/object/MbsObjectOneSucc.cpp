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
 * \file mbslib/elements/object/MbsObjectOneSucc.cpp
 * Definition of mbslib ::MbsObjectOneSucc
 */

#include <mbslib/elements/object/MbsObjectOneSucc.hpp>

using namespace mbslib;

bool MbsObjectOneSucc::isValid() const {
    return (MbsObject::isValid() && (succ != NULL));
}

MbsObjectOneSucc::MbsObjectOneSucc(const std::string & n)
    : MbsObject(n)
    , succ(NULL) {
}

MbsObjectOneSucc::MbsObjectOneSucc(MbsObject & p, const std::string & n)
    : MbsObject(p, n)
    , succ(NULL) {
}

const MbsObject * MbsObjectOneSucc::getSuccessor(size_t index) const {
    if (index == 0)
        return this->succ;
    return 0;
}

bool MbsObjectOneSucc::addSuccessor(MbsObject & s) {
    if (succ) {
        setupError = true;
        return false;
    } else {
        succ = &s;
        return true;
    }
}

size_t MbsObjectOneSucc::getSuccessorCount() const {
    return 1;
}

MbsObjectOneSucc::~MbsObjectOneSucc() {
}
