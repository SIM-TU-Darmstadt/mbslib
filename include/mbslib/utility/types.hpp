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
 * \file mbslib/utility/types.hpp
 * 
 */
#ifndef __MBSLIB_TYPES_HPP__
#define __MBSLIB_TYPES_HPP__

#ifdef EIGEN_WORLD_VERSION
#error "Eigen header files must be included after mbslib header files!"
#endif

// vc does not have isnan - it does not belong to the c++ standard
// however defining the function in the namespace std does not work either
// i used this workaround.

#ifndef USE_SINGLE
#ifndef USE_DOUBLES
#define USE_DOUBLES
#endif
#endif

#include <vector>
#ifdef USE_ADOLC

#include <mbslib/utility/types_adolc.hpp>
namespace mbslib {
typedef adouble TScalar;
}
#define REAL_MAX DBL_MAX
#define REAL_MIN DBL_MIN

#else

#include <Eigen/Core>
#include <math.h>
#include <float.h>
#ifdef USE_DOUBLES
namespace mbslib {
typedef double TScalar;
}
#define REAL_MAX DBL_MAX
#define REAL_MIN DBL_MIN
#elif USE_SINGLE
namespace mbslib {
typedef float TScalar;
}
#define REAL_MAX FLT_MAX
#define REAL_MIN FLT_MIN
#endif
typedef mbslib::TScalar Real;

void condassign(mbslib::TScalar & a, mbslib::TScalar b, mbslib::TScalar c, mbslib::TScalar d);

#endif

// on win32 platforms certain compliers (e.g. VS2010) don't know a predefined M_PI:
// Note actually they do: in <math.hpp>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_4
#define M_PI_4 0.785398163397448309616
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#ifndef M_EPS
#define M_EPS 10e-6
#endif

#define EPSILON 1.0E-9

namespace mbslib {
typedef TScalar TTime;

typedef Eigen::Matrix< TScalar, 3, 1 > TVector3;
typedef Eigen::Matrix< TScalar, 6, 1 > TVector6;
typedef Eigen::Matrix< TScalar, 3, 3 > TMatrix3x3;
typedef Eigen::Matrix< TScalar, 6, 6 > TMatrix6x6;

typedef Eigen::Matrix< TScalar, Eigen::Dynamic, 1 > TVectorX;
typedef Eigen::Matrix< TScalar, Eigen::Dynamic, Eigen::Dynamic > TMatrixX;
typedef Eigen::Matrix< TScalar, 6, Eigen::Dynamic > TMatrix6xX;

typedef std::vector< TVector3, Eigen::aligned_allocator< TVector3 > > TVector3Vector;
typedef std::vector< TVector6, Eigen::aligned_allocator< TVector6 > > TVector6Vector;
typedef std::vector< TVectorX, Eigen::aligned_allocator< TVectorX > > TVectorXVector;
typedef std::vector< TMatrix3x3, Eigen::aligned_allocator< TMatrix3x3 > > TMatrix3x3Vector;
typedef std::vector< TMatrix6x6, Eigen::aligned_allocator< TMatrix6x6 > > TMatrix6x6Vector;
typedef std::vector< TMatrixX, Eigen::aligned_allocator< TMatrixX > > TMatrixXVector;

/*
template<int T>
struct FixedSizeTypes{
  typedef Eigen::Matrix<TScalar,T,1> TVectorN;
  typedef Eigen::Matrix<TScalar,6,T> TMatrix6xN;
};
*/

enum class JointType {
    Prismatic,
    Revolute
};
}

#include <mbslib/utility/AggregateBody.hpp>
#include <mbslib/utility/CoordinateFrame.hpp>
#include <mbslib/utility/ForceTorque.hpp>
#include <mbslib/utility/JointState.hpp>

#endif
