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
 * \file mbslib/elements/drive/ActiveDrive.hpp
 * Declaration of mbslib ::ActiveDrive
 */
#ifndef __MBSLIB_ACTIVEDRIVE_HPP__
#define __MBSLIB_ACTIVEDRIVE_HPP__

#include <mbslib/elements/drive/Drive.hpp>

namespace mbslib {

/**
 * \brief Active drive.
 */
class ActiveDrive : public Drive {
public:
    /**
   * \brief Set desired position.
   *
   * \param q_des The desired joint positions.
   */
    void setDesiredPosition(TScalar q_des);

    /**
   * \brief Set desired velocity.
   *
   * \param dq_des  The desired joint velocities.
   */
    void setDesiredVelocity(TScalar dq_des);
    virtual ~ActiveDrive();

protected:
    /**
   * \brief Constructor.
   *
   * \param joint The joint.
   * \param name  (optional) the name.
   */
    ActiveDrive(Joint1DOF & joint, const std::string & name = "");

    /// Desired position.
    TScalar q_des;

    /// Desired velocity.
    TScalar dq_des;

    CONTINUE_PARAMETER_LIST(Drive)
    PARAMETER(q, q_des, q_des);
    PARAMETER(dq, dq_des, dq_des);
    END_PARAMETER_LIST
}; // class ActiveDrive
}

#endif
