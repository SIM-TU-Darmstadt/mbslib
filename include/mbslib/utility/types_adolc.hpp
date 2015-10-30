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
 * \file mbslib/utility/types_adolc.hpp
 *
 */
#ifndef __MBSLIB_TYPES_ADOLC_HPP__
#define __MBSLIB_TYPES_ADOLC_HPP__

#ifdef USE_ADOLC

#include <Eigen/Core>
#include <Eigen/src/Core/MathFunctions.h>
#include <cmath>
#include <limits>
#include <random>

#ifdef USE_ADOLC_TAPELESS

#if EIGEN_WORLD_VERSION >= 3

#ifndef ADOLCSUPPORT_H
#define ADOLCSUPPORT_H
#define ADOLC_TAPELESS
#include <adolc/adouble.h>

typedef adtl::adouble adouble;

namespace std {
double ceil(adouble & a) {
    return std::ceil(a.getADValue());
}
}
inline double ceil(adouble & a) {
    return std::ceil(a.getADValue());
}

namespace Eigen {
template <>
struct NumTraits< adtl::adouble >
    : NumTraits< double > // permits to get the epsilon, dummy_precision, lowest, highest functions
{
    typedef adtl::adouble Real;
    typedef adtl::adouble NonInteger;
    typedef adtl::adouble Nested;
    enum {
        IsComplex = 0,
        IsInteger = 0,
        IsSigned = 1,
        RequireInitialization = 1,
        ReadCost = 1,
        AddCost = 3,
        MulCost = 3
    };
};
}
namespace adtl {
inline const adouble & conj(const adouble & x) {
    return x;
}
inline const adouble & real(const adouble & x) {
    return x;
}
inline adouble imag(const adouble &) {
    return 0.;
}
inline adouble abs(const adouble & x) {
    return fabs(x);
}
inline adouble abs2(const adouble & x) {
    return x * x;
}
}
#endif // ADOLCSUPPORT_H
#endif

#else

///################################################
/// ADOL-C Tape mode
///################################################

#include <adolc/adolc.h>

extern adub ceil(const badouble & a);
//adub ceil( adub & a);
extern adub fabs(const badouble & x);
extern adouble abs(const adouble & x);
extern adub sqrt(const badouble & x);
extern adub sin(const badouble & x);
extern adub cos(const badouble & x);
extern adub log(const badouble & x);
//adub sqrt(const adub& x);

namespace std {
adouble ceil(const badouble & a);
//adub ceil( adub & a);
adouble abs(const badouble & x);
//adub abs(const adub & x);
adouble sqrt(const badouble & x);
//adub sqrt(const adub& x);
}

#if EIGEN_WORLD_VERSION >= 3

#include <Eigen/src/Core/MathFunctions.h>

namespace Eigen {
namespace internal {
template <>
struct cast_impl< adouble, int > {
    static inline int run(const adouble & x) {
        return static_cast< int >(x.getValue());
    }
};
}
}

namespace std {
template <>
struct numeric_limits< adouble > {
    static constexpr bool is_specialized = true;
    adouble epsilon() {
        return 1e-11;
    }
    static constexpr bool is_integer = false;
    static constexpr bool is_signed = true;
};
}
#endif

//IMPORTANT NOTE:
//It is not absolutely clear to me (MF), were the ei_* functions have to be
//defined to be found in a proper way. It seems to be dependent on the IDE
//wether this hast to be done within namespace Eigen or not.
//Some future research is required to make this generally applicable!

namespace Eigen {
template <>
struct NumTraits< adouble > {
    typedef adouble Real;
    typedef adouble FloatingPoint;
    typedef adouble Nested;
    enum {
        IsComplex = 0,
        IsInteger = 0,
        RequireInitialization = 1,
        HasFloatingPoint = 1,
        ReadCost = 1,
        AddCost = 1,
        MulCost = 1
    };
    inline static Real epsilon() {
        return 1e-11;
    }
    inline static Real dummy_precision() {
        return 2.220e-16;
    }
};

#if EIGEN_WORLD_VERSION < 3
template <>
inline adouble precision< adouble >() {
    return 1e-11;
}
template <>
inline adouble machine_epsilon< adouble >() {
    return 2.220e-16;
}
#endif //EIGEN_WORLD_VERSION < 3

//inline adouble ei_real(const adouble & x)  { return x; }
//inline adouble ei_imag(const adouble & x)    { return 0.; }
//inline adouble ei_conj(const adouble & x)  { return x; }
inline adouble ei_abs(const adouble & x) {
    return (x < 0) ? (-x) : (x);
}
//inline adouble ei_abs2(const adouble & x)  { return x*x; }
//inline adouble ei_sqrt(const adouble & x)  { return sqrt(x); }
//inline adouble ei_exp(const adouble & x)   { return exp(x); }
inline adouble ei_log(const adouble & x) {
    return log(x);
}
//inline adouble ei_sin(const adouble & x)   { return sin(x); }
//inline adouble ei_cos(const adouble & x)   { return cos(x); }
//inline adouble ei_atan2(const adouble & y, const adouble & x) { return atan2(y,x); }
//inline adouble ei_pow(const adouble & x, const adouble & y) { return pow(x, y); }
template < typename T >
inline T ei_random() {
    return adouble(ei_random< T >());
}

#if EIGEN_WORLD_VERSION >= 3
template <>
inline double ei_random() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return std::generate_canonical< double, 64 >(gen);
}
#endif
template <>
inline adouble ei_random() {
    return adouble(ei_random< double >());
}

#if EIGEN_WORLD_VERSION >= 3
inline bool ei_isMuchSmallerThan(const adouble & a, const adouble & b, const adouble prec = 1e-11) {
    return (ei_abs(a) <= (ei_abs(b) * prec)) != 0; // note: operator<= in ADOL-C is of type int, we use the expression here to avoid a warning in MSVS
}
#else
//    inline bool ei_isMuchSmallerThan(const adouble & a, const adouble & b, const adouble prec = precision<adouble>())
//    {
//        return ( Eigen::ei_abs(a) <= ( Eigen::ei_abs(b) * prec ) ) != 0; // note: operator<= in ADOL-C is of type int, we use the expression here to avoid a warning in MSVS
//    }
#endif //EIGEN_WORLD_VERSION >= 3

} // namespace Eigen

//template<typename T> inline T ei_random();
//template<> inline double ei_random();
//template<> inline adouble ei_random() { return adouble(ei_random<double>()); }
inline adouble ei_real(const adouble & x) {
    return x;
}
//template adouble ei_random<adouble>() { return adouble(Eigen::ei_random<double>()); }
//inline adouble ei_imag(const adouble & x)    { return 0.; }
inline adouble ei_conj(const adouble & x) {
    return x;
}
inline adouble ei_abs(const adouble & x) {
    return ::abs(x); /*(x<0)?(-x):(x);*/
}
inline adouble ei_abs2(const adouble & x) {
    return x * x;
}
inline adouble ei_sqrt(const adouble & x) {
    return ::sqrt(x);
}
//inline adouble ei_exp(const adouble & x)   { return exp(x); }
inline adouble ei_log(const adouble & x) {
    return ::log(x);
}
inline adouble ei_sin(const adouble & x) {
    return ::sin(x);
}
inline adouble ei_cos(const adouble & x) {
    return ::cos(x);
}
//inline adouble ei_atan2(const adouble & y, const adouble & x) { return atan2(y,x); }
//inline adouble ei_pow(const adouble & x, const adouble & y) { return pow(x, y); }
inline bool ei_isMuchSmallerThan(const adouble & a, const adouble & b, const adouble prec = 1e-11) {
    return (::ei_abs(a) <= (::ei_abs(b) * prec)) != 0; // note: operator<= in ADOL-C is of type int, we use the expression here to avoid a warning in MSVS
    //return Eigen::ei_isMuchSmallerThan(a,b,prec);
}

///################################################
/// ADOL-C Tape mode end
///################################################

#endif //USE_ADOLC_TAPELESS

#endif // __MBSLIB_TYPES_ADOLC_HPP__

#endif // USE_ADOLC
