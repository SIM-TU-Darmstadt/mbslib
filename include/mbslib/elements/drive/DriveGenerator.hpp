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
 * \file mbslib/elements/drive/DriveGenerator.hpp
 * Declaration of mbslib::DriveGenerator
 */
#ifndef __MBSLIB_DRIVEGENERATOR_HPP__
#define __MBSLIB_DRIVEGENERATOR_HPP__

#include <mbslib/elements/drive/Drive.hpp>
#include <string>
namespace mbslib {

/**
 * \brief Drive generator.
 */
class DriveGenerator {
public:
    virtual ~DriveGenerator() {
    }
    /**
   * \brief Generate a drive which is attached to the given joint.
   *  
   * \param joint The joint.
   * \param name  (optional) the name.
   *
   * \return  null if it fails, else the drive.
   */
    virtual Drive * generateDrive(Joint1DOF & joint, const std::string & name = "") const = 0;
};
}

#endif
