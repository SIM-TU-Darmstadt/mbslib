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
 * \file mbslib/elements/drive/Drive.hpp
 * Declaration of mbslib ::Drive
 */
#ifndef __MBSLIB_DRIVE_HPP__
#define __MBSLIB_DRIVE_HPP__

#include <mbslib/parameter/ParametrizedObject.hpp>
#include <mbslib/utility/types.hpp>
#include <mbslib/elements/joint/Joint1DOF.hpp>

namespace mbslib {

/**
 * \brief Drive.
 */
class Drive : public ParametrizedObject {
public:
    /**
   * \brief Check if setup is valid.
   *
   * \return  true if valid, false if not.
   */
    virtual bool isValid() const;

    /**
   * \brief Drive the attached joint.
   */
    virtual void doForwardDrive() = 0;

    /**
   * \brief Drive the attached joint.
   */
    virtual void doInverseDrive() = 0;

    Joint1DOF * getAttachedJoint() const;

protected:
    virtual ~Drive();
    /**
   * \brief Constructor.
   *
   * \param joint The joint.
   * \param name  (optional) the name.
   */
    Drive(Joint1DOF & joint, const std::string & name = "");

    /// The joint driven by this drive.
    Joint1DOF * attachedJoint;

    EMPTY_PARAMETER_LIST
}; // class Drive
}

#endif
