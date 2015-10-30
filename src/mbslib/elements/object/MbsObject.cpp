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
 * \file mbslib/elements/object/MbsObject.cpp
 * Definition of mbslib ::MbsObject
 */

#include <mbslib/elements/MbsObject.hpp>
#include <mbslib/elements/joint/Joint.hpp>

using namespace mbslib;

MbsObject::~MbsObject() {
}

MbsObject::MbsObject(const std::string & n)
    : ParametrizedObject(n)
    //,name(n)
    , pred(NULL)
    , setupError(false) {
    relr.setZero();
    relR.setIdentity();
    addParameter("rX", cof.r(0));
    addParameter("rY", cof.r(1));
    addParameter("rZ", cof.r(2));
}

MbsObject::MbsObject(MbsObject & p, const std::string & n)
    : ParametrizedObject(n)
    //,name(n)
    , pred(&p)
    , setupError(false) {
    p.addSuccessor(*this);
    relr.setZero();
    relR.setIdentity();
    addParameter("rX", cof.r(0));
    addParameter("rY", cof.r(1));
    addParameter("rZ", cof.r(2));
}

void MbsObject::doDirkin() {
    doPosition();
    doVelocity();
    doAcceleration();
}

void MbsObject::doPosition() {
    calcRelPose(relr, relR);
    cof.R = pred->getCoordinateFrame().R * relR;
    cof.r = pred->getCoordinateFrame().r + cof.R * relr;
}

void MbsObject::doABASweep1hyb(const std::vector< bool > &) {
    // for all none-joints fwd and inv of ABA are the same as S == 0
    this->doABASweep1fwd();
    // make sure, that this is no joint
    assert(dynamic_cast< Joint * >(this) == NULL);
}

void MbsObject::doABASweep2hyb(const std::vector< bool > &) {
    // for all none-joints fwd and inv of ABA are the same as S == 0
    this->doABASweep2fwd();
    // make sure, that this is no joint
    assert(dynamic_cast< Joint * >(this) == NULL);
}

void MbsObject::doABASweep3hyb(const std::vector< bool > &) {
    // for all none-joints fwd and inv of ABA are the same as S == 0
    this->doABASweep3fwd();
    // make sure, that this is no joint
    assert(dynamic_cast< Joint * >(this) == NULL);
}

bool MbsObject::addSuccessor(MbsObject & o) {
    setupError = true;
    return false;
}

void MbsObject::calculateJacobian(TMatrix6xX & jacobian, const TVector3 & referencePoint) const {
    makeJacobian(jacobian, referencePoint);
    if (pred) {
        pred->calculateJacobian(jacobian, referencePoint);
    }
}
