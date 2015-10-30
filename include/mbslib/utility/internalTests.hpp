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
 * \file mbslib/utility/internalTests.hpp
 *
 */
#ifndef __MBSLIB_INTERNALTESTS_HPP__
#define __MBSLIB_INTERNALTESTS_HPP__

#include <mbslib/utility/types.hpp>

#include <assert.h>
namespace mbslib {

/**
 * \brief checks if inertia is valid.
 *
 * \param m     The mass.
 * \param I     The inertia tensor.
 * \param name  (optional) the name.
 * \param place (optional) the place. \todo kommentieren
 * \param line  (optional) the line. \todo kommentieren
 *
 * \return  true if it succeeds, false if it fails.
 */
bool validInertia(TScalar m, const TMatrix3x3 & I, const std::string & name = "", const std::string & place = "", int line = 0);

/**
 * \brief checks the unit vector.
 *
 * \param v     The vector.
 * \param name  (optional) the name.
 * \param place (optional) the place.
 * \param line  (optional) the line.
 *
 * \return  true if it succeeds, false if it fails.
 */
bool unitVector(const TVector3 & v, const std::string & name = "", const std::string & place = "", int line = 0);

/**
 * \brief checks the unit vector.
 *
 * \param v     The vector.
 * \param name  (optional) the name.
 * \param place (optional) the place.
 * \param line  (optional) the line.
 *
 * \return  true if it succeeds, false if it fails.
 */
bool unitVector(const TVector6 & v, const std::string & name = "", const std::string & place = "", int line = 0);

/**
 * \brief checks a value if it is not near zero.
 *
 * \param s     The scalar to check.
 * \param name  (optional) the name.
 * \param place (optional) the place.
 * \param line  (optional) the line.
 *
 * \return  true if it succeeds, false if it fails.
 */
bool notNearZero(TScalar s, const std::string & name = "", const std::string & place = "", int line = 0);

} //namespace mbslib

/**
 * \brief Check inertia.
 *
 * \param m The mass.
 * \param I The inertia tensor
 */
#ifdef NDEBUG
#define checkInertia(m, I)
#else
#define checkInertia(m, I) validInertia(m, I, std::string(#m " and " #I), __FILE__, __LINE__);
#endif

/**
 * \brief Check unit vector.
 *
 * \param X The vector to check.
 */
#ifdef NDEBUG
#define checkUnitVector(X)
#else
#define checkUnitVector(X) unitVector(X, std::string(#X), __FILE__, __LINE__)
#endif

/**
 * \brief Check not zero.
 *
 * \param X the scalar to check
 */
#ifdef NDEBUG
#define checkNotZero(X)
#else
#define checkNotZero(X) notNearZero(X, std::string(#X), __FILE__, __LINE__);
#endif

#endif
