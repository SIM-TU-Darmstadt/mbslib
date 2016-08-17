/*
 * Copyright (C) 2016
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

#ifndef __MBSLIB_DLRLBR2_HPP__
#define __MBSLIB_DLRLBR2_HPP__

#include <mbslib/mbslib.hpp>

mbslib::TVectorX DLRLBR2_C(const mbslib::TVectorX & q, const mbslib::TVectorX & qdot);
mbslib::TVectorX DLRLBR2_G(const mbslib::TVectorX & q);
mbslib::TMatrixX DLRLBR2_M(const mbslib::TVectorX & q);
mbslib::TVector3 DLRLBR2_DIRKIN_xyz(const mbslib::TMatrixX & q);

#endif // __MBSLIB_DLRLBR2_HPP__
