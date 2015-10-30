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
 * \file mbslib/collision/PointContactHandler.hpp
 * Declaration of mbslib::PointContactHandler
 */
#ifndef __MBSLIB_POINTCONTACTHANDLER_HPP__
#define __MBSLIB_POINTCONTACTHANDLER_HPP__

#include <mbslib/collision/PointContact.hpp>

namespace mbslib {

/**
 * \brief a Point contact handler.
 */
class PointContactHandler {
public:
    /**
   * \brief Constructor.
   *
   * \param k \todo kommentieren
   * \param d \todo kommentieren
   */
    PointContactHandler(TScalar k, TScalar d);

    /**
   * \brief Apply force at contact.
   */
    void applyForce(PointContact &);

    /**
   * \brief Reset force at contact.
   */
    void resetForce(PointContact &);

    virtual ~PointContactHandler();

protected:
    /// The k \todo kommentieren
    TScalar k;
    /// The d \todo kommentieren
    TScalar d;

}; // class PointContactHandler

} // namespace mbslib

#endif
