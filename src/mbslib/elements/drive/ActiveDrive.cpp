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
 * \file mbslib/elements/drive/ActiveDrive.cpp
 * Definition of mbslib ::ActiveDrive
 */

#include <mbslib/elements/drive/ActiveDrive.hpp>

using namespace mbslib;

void ActiveDrive::setDesiredPosition(TScalar q_des) {
    this->q_des = q_des;
}

void ActiveDrive::setDesiredVelocity(TScalar dq_des) {
    this->dq_des = dq_des;
}

ActiveDrive::ActiveDrive(Joint1DOF & joint, const std::string & name)
    : Drive(joint, name)
    , q_des(0)
    , dq_des(0) {
}
ActiveDrive::~ActiveDrive() {
}
