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
 * \file mbslib/elements/drive/PassiveSpringDamperDrive.hpp
 * Declaration of mbslib ::PassiveSpringDamperDrive
 */
#ifndef __MBSLIB_PASSIVESPRINGDAMPERDRIVE_HPP__
#define __MBSLIB_PASSIVESPRINGDAMPERDRIVE_HPP__

#include <mbslib/elements/drive/PassiveDrive.hpp>

namespace mbslib {

/**
 * \brief Passive spring damper drive.
 */
class PassiveSpringDamperDrive : public PassiveDrive {
public:
    /**
   * \brief Constructor.
   *
   * \param joint           The joint.
   * \param springCoeff     The spring coefficient.\todo kommentieren
   * \param damperCoeff     The damper coefficient.
   * \param name            (optional) the name.
   */
    PassiveSpringDamperDrive(Joint1DOF & joint, TScalar springCoeff, TScalar damperCoeff, const std::string & name = "");

    /**
   * \brief Drive the attached joint.
   */
    virtual void doForwardDrive();
    virtual void doInverseDrive();
    virtual ~PassiveSpringDamperDrive();

protected:
    /// spring coefficient
    TScalar springCoeff;

    /// dampener coefficient
    TScalar damperCoeff;

    CONTINUE_PARAMETER_LIST(PassiveDrive)
    PARAMETER(springconstant, springCoeff, springCoeff)
    PARAMETER(springdamping, damperCoeff, damperCoeff)
    END_PARAMETER_LIST
}; // class PassiveSpringDamperDrive
}

#endif
