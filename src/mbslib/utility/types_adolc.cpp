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

#ifdef USE_ADOLC

#include <mbslib/utility/types_adolc.hpp>

#ifdef USE_ADOLC_TAPELESS
#if EIGEN_WORLD_VERSION >= 3

#else // EIGEN_WORLD_VERSION>=3

#endif // EIGEN_WORLD_VERSION>=3
#else  // USE_ADOLC_TAPELESS
#include <adolc/adolc.h>
#include <adolc/adouble.h>
#include <adolc/interfaces.h>

adouble abs(const adouble & x) {
    return ::fabs(x);
}

namespace std {
adouble ceil(const badouble & a) {
    return ::ceil(a);
}
//adub ceil( adub & a){return ::ceil( adouble(a) ); }
adouble abs(const badouble & x) {
    return ::fabs(x);
}
adouble sqrt(const badouble & x) {
    return ::sqrt(x);
}
}

#if EIGEN_WORLD_VERSION >= 3

#else // EIGEN_WORLD_VERSION>=3

#endif // EIGEN_WORLD_VERSION>=3
#endif // USE_ADOLC_TAPELESS

#endif // USE_AOLDC
