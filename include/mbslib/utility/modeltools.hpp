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
 * \file mbslib/utility/modeltools.hpp
 * 
 */
#ifndef __MBSLIB_MODELTOOLS_HPP__
#define __MBSLIB_MODELTOOLS_HPP__

#include <mbslib/utility/types.hpp>

namespace mbslib {

/**
 * \brief Makes an inertia tensor.
 *
 * \param xx  The xx component.
 * \param yy  The yy component.
 * \param zz  The zz component.
 * \param xy  (optional) the xy component.
 * \param xz  (optional) the xz component.
 * \param yz  (optional) the yz component.
 *
 * \return The inertia tensor.
 */
TMatrix3x3 makeInertiaTensor(TScalar xx, TScalar yy, TScalar zz, TScalar xy = 0, TScalar xz = 0, TScalar yz = 0);

/**
 * \brief Makes an inertia tensor of a cylinder around the z axis.
 *
 * \param radius  The radius.
 * \param length  The length.
 * \param mass    The mass.
 *
 * \return inertia tensor .
 */
TMatrix3x3 makeInertiaTensorCylinderZ(TScalar radius, TScalar length, TScalar mass);

/**
 * \brief Makes an inertia tensor of a cylinder around the y axis.
 *
 * \param radius  The radius.
 * \param length  The length.
 * \param mass    The mass.
 *
 * \return  inertia tensor.
 */
TMatrix3x3 makeInertiaTensorCylinderY(TScalar radius, TScalar length, TScalar mass);

/**
 * \brief Makes an inertia tensor of a cylinder around the x axis.
 *
 * \param radius  The radius.
 * \param length  The length.
 * \param mass    The mass.
 *
 * \return  inertia tensor.
 */
TMatrix3x3 makeInertiaTensorCylinderX(TScalar radius, TScalar length, TScalar mass);

} //namespace mbslib

#endif
