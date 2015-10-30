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
 * \file mbslib/elements/object/MbsObjectNoSucc.cpp
 * Definition of mbslib ::MbsObjectNoSucc
 */

#include <mbslib/elements/object/MbsObjectNoSucc.hpp>

using namespace mbslib;

MbsObjectNoSucc::MbsObjectNoSucc(const std::string & n)
    : MbsObject(n) {
}

MbsObjectNoSucc::MbsObjectNoSucc(MbsObject & p, const std::string & n)
    : MbsObject(p, n) {
}

bool MbsObjectNoSucc::addSuccessor(MbsObject &) {
    setupError = true;
    return false;
}

size_t MbsObjectNoSucc::getSuccessorCount() const {
    return 0;
}

const MbsObject * MbsObjectNoSucc::getSuccessor(size_t /*i*/) const {
    return 0;
}

MbsObjectNoSucc::~MbsObjectNoSucc() {
}
