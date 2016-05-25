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
 * \file mbslib/elements/drive/ActiveSDDriveGenerator.cpp
 * Definition of mbslib::ActiveSDDriveGenerator
 */

#include <mbslib/elements/drive/ActiveSDDriveGenerator.hpp>
#include <mbslib/elements/drive/ActiveSpringDamperDrive.hpp>

mbslib::ActiveSDDriveGenerator::ActiveSDDriveGenerator(TScalar stiffness, TScalar friction)
    : stiffness(stiffness)
    , friction(friction) {
}

mbslib::Drive * mbslib::ActiveSDDriveGenerator::generateDrive(Joint1DOF & joint, const std::string & name) const {
    return new ActiveSpringDamperDrive(joint, stiffness, friction, name);
}
mbslib::ActiveSDDriveGenerator::~ActiveSDDriveGenerator() {
}