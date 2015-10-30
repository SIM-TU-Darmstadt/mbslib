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
 * \file mbslib/elements/drive/ActiveSpringDamperDrive.hpp
 * Declaration of mbslib ::ActiveSpringDamperDrive
 */
#ifndef __MBSLIB_ACTIVESPRINGDAMPERDRIVE_HPP__
#define __MBSLIB_ACTIVESPRINGDAMPERDRIVE_HPP__

#include <mbslib/elements/drive/ActiveDrive.hpp>

namespace mbslib {

/**
 * \brief Active spring damper drive.
 */
class ActiveSpringDamperDrive : public ActiveDrive {
public:
    /**
   * \brief Constructor.
   *
   * \param joint           The joint.
   * \param springCoeff     The spring coefficient. \todo kommentieren
   * \param damperCoeff     The damper coefficient.
   * \param name            (optional) the name.
   */
    ActiveSpringDamperDrive(Joint1DOF & joint, TScalar springCoeff, TScalar damperCoeff, const std::string & name = "");

    /**
   * \brief Drive the attached joint.
   */
    virtual void doForwardDrive();
    virtual void doInverseDrive();
    virtual ~ActiveSpringDamperDrive();

protected:
    /// spring coefficient
    TScalar springCoeff;

    /// damper coefficient
    TScalar damperCoeff;

    CONTINUE_PARAMETER_LIST(ActiveDrive)
    PARAMETER(springconstant, springCoeff, springCoeff)
    PARAMETER(springdamping, damperCoeff, damperCoeff)
    END_PARAMETER_LIST

}; // class ActiveSpringDamperDrive
}

#endif
