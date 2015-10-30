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
 * \file mbslib/utility/mathtools.hpp
 * 
 */
#ifndef __MBSLIB_MATHTOOLS_HPP__
#define __MBSLIB_MATHTOOLS_HPP__

#include <mbslib/utility/types.hpp>
namespace mbslib {

/**
   * \brief Makes a crossproduct matrix.
   *
   * \param v The input vector.
   *
   * \return the crossproduct matrix .
   */
TMatrix3x3 makeCrossproductMatrix(const TVector3 & v);

// Note for the following 6 transform function:
// we have a spatial vector given in COF B
// COF B's coordinates are given relative to COF A
// the generated matrices will calculate the equivalent
// velocity/force at A

/**
   * \brief Makes a force transform.
   *
   * \param aRb \todo kommentieren
   * \param arb \todo kommentieren
   *
   * \return \todo kommentieren
   */
TMatrix6x6 makeForceTransform(const TMatrix3x3 & aRb, const TVector3 & arb);

/**
   * \brief Makes a force transform.
   *
   * \param abr \todo kommentieren
   *
   * \return  \todo kommentieren
   */
TMatrix6x6 makeForceTransform(const TVector3 & abr);

/**
   * \brief Makes a force transform.
   *
   * \param aRb \todo kommentieren
   *
   * \return  \todo kommentieren
   */
TMatrix6x6 makeForceTransform(const TMatrix3x3 & aRb);

/**
   * \brief Makes a velocity transform.
   *
   * \param aRb \todo kommentieren
   * \param arb \todo kommentieren
   *
   * \return  \todo kommentieren
   */
TMatrix6x6 makeVelocityTransform(const TMatrix3x3 & aRb, const TVector3 & arb);

/**
   * \brief Makes a velocity transform.
   *
   * \param arb \todo kommentieren
   *
   * \return  \todo kommentieren
   */
TMatrix6x6 makeVelocityTransform(const TVector3 & arb);

/**
   * \brief Makes a velocity transform.
   *
   * \param aRb \todo kommentieren
   *
   * \return  \todo kommentieren
   */
TMatrix6x6 makeVelocityTransform(const TMatrix3x3 & aRb);

/**
   * \brief Transform force.
   *
   * \param [out]     target  \todo kommentieren
   * \param aRb               \todo kommentieren
   * \param v                 \todo kommentieren
   */
void transformForce(TVector6 & target, const TMatrix3x3 & aRb, const TVector6 & v);

/**
   * \brief Transform force.
   *
   * \param [out]     target  \todo kommentieren
   * \param arb               \todo kommentieren
   * \param v                 \todo kommentieren
   */
void transformForce(TVector6 & target, const TVector3 & arb, const TVector6 & v);

/**
   * \brief Transform force.
   *
   * \param [out]     target  \todo kommentieren
   * \param aRb               \todo kommentieren
   * \param arb               \todo kommentieren
   * \param v                 \todo kommentieren
   */
void transformForce(TVector6 & target, const TMatrix3x3 & aRb, const TVector3 & arb, const TVector6 & v);

/**
   * \brief Transform motion.
   *
   * \param [out]     target  \todo kommentieren
   * \param aRb               \todo kommentieren
   * \param v                 \todo kommentieren
   */
void transformMotion(TVector6 & target, const TMatrix3x3 & aRb, const TVector6 & v);

/**
   * \brief Transform motion.
   *
   * \param [out]  target    \todo kommentieren
   * \param arb              \todo kommentieren
   * \param v                \todo kommentieren
   */
void transformMotion(TVector6 & target, const TVector3 & arb, const TVector6 & v);

/**
   * \brief Transform motion.
   *
   * \param [out]  target     \todo kommentieren
   * \param aRb               \todo kommentieren
   * \param arb               \todo kommentieren
   * \param v                 \todo kommentieren
   */
void transformMotion(TVector6 & target, const TMatrix3x3 & aRb, const TVector3 & arb, const TVector6 & v);

/**
   * \brief Transform a bi.
   *
   * \param [out] target  \todo kommentieren
   * \param aRb           \todo kommentieren
   * \param abi           \todo kommentieren
   */
void transformABI(TMatrix6x6 & target, const TMatrix3x3 & aRb, const TMatrix6x6 & abi);

/**
   * \brief Transform a bi.
   *
   * \param [out] target  \todo kommentieren
   * \param arb           \todo kommentieren
   * \param abi           \todo kommentieren
   */
void transformABI(TMatrix6x6 & target, const TVector3 & arb, const TMatrix6x6 & abi);

/**
   * \brief Transform a bi.
   *
   * \param [out]  target     \todo kommentieren
   * \param aRb               \todo kommentieren
   * \param arb               \todo kommentieren
   * \param abi               \todo kommentieren
   */
void transformABI(TMatrix6x6 & target, const TMatrix3x3 & aRb, const TVector3 & arb, const TMatrix6x6 & abi);

/**
   * \brief Spatial cross.
   *
   * \param v1  The first spatial vector
   * \param v2  The second spatial vector
   *
   * \return  the spatial cross of v1 and v2.
   */
TVector6 spatialCross(const TVector6 & v1, const TVector6 & v2);

/**
   * \brief Spatial cross star.
   *
   * \param v1  The first spatial vector.
   * \param v2  The second spatial vector.
   *
   * \return  the spatial corss star of v1 and v2.
   */
TVector6 spatialCrossStar(const TVector6 & v1, const TVector6 & v2);

/**
   * \brief Mirror  thevector.
   *
   * \param v       The vector.
   * \param normal  The normal.
   *
   * \return  the mirrored vector.
   */
TVector3 mirrorVector(const TVector3 & v, const TVector3 & normal);
}
#endif
