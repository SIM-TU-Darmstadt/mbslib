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
 * \file mbslib/utility/mathtools.cpp
 *
 */
#include <mbslib/utility/mathtools.hpp>

#include <Eigen/Geometry>
using namespace mbslib;
TMatrix3x3 mbslib::makeCrossproductMatrix(const TVector3 & v) {
    TMatrix3x3 m;
    m(0, 0) = 0.0;
    m(0, 1) = -v(2);
    m(0, 2) = v(1);
    m(1, 0) = v(2);
    m(1, 1) = 0.0;
    m(1, 2) = -v(0);
    m(2, 0) = -v(1);
    m(2, 1) = v(0);
    m(2, 2) = 0.0;
    return m;
}

TMatrix6x6 mbslib::makeForceTransform(const TVector3 & r) {
    return makeForceTransform(TMatrix3x3::Identity(), r);
}

TMatrix6x6 mbslib::makeForceTransform(const TMatrix3x3 & R) {
    return makeForceTransform(R, TVector3::Zero());
}

TMatrix6x6 mbslib::makeForceTransform(const TMatrix3x3 & R, const TVector3 & r) {
    TMatrix6x6 m;
    m.block< 3, 3 >(0, 0) = R;
    m.block< 3, 3 >(0, 3) = makeCrossproductMatrix(r) * R;
    m.block< 3, 3 >(3, 0).setZero();
    m.block< 3, 3 >(3, 3) = R;
    return m;
}

TMatrix6x6 mbslib::makeVelocityTransform(const TVector3 & r) {
    return makeVelocityTransform(TMatrix3x3::Identity(), r);
}

TMatrix6x6 mbslib::makeVelocityTransform(const TMatrix3x3 & R) {
    return makeVelocityTransform(R, TVector3::Zero());
}

TMatrix6x6 mbslib::makeVelocityTransform(const TMatrix3x3 & aRb, const TVector3 & arb) {
    TMatrix6x6 m;
    m.block< 3, 3 >(0, 0) = aRb;
    m.block< 3, 3 >(0, 3).setZero();
    m.block< 3, 3 >(3, 0) = makeCrossproductMatrix(arb) * aRb;
    m.block< 3, 3 >(3, 3) = aRb;
    return m;
}

void mbslib::transformForce(TVector6 & target, const TMatrix3x3 & aRb, const TVector6 & v) {
    target = makeForceTransform(aRb) * v;
}

void mbslib::transformForce(TVector6 & target, const TVector3 & arb, const TVector6 & v) {
    target = makeForceTransform(arb) * v;
}

void mbslib::transformForce(TVector6 & target, const TMatrix3x3 & aRb, const TVector3 & arb, const TVector6 & v) {
    target = makeForceTransform(aRb, arb) * v;
}

void mbslib::transformMotion(TVector6 & target, const TMatrix3x3 & aRb, const TVector6 & v) {
    target = makeVelocityTransform(aRb) * v;
}

void mbslib::transformMotion(TVector6 & target, const TVector3 & arb, const TVector6 & v) {
    target = makeVelocityTransform(arb) * v;
}

void mbslib::transformMotion(TVector6 & target, const TMatrix3x3 & aRb, const TVector3 & arb, const TVector6 & v) {
    target = makeVelocityTransform(aRb, arb) * v;
}

void mbslib::transformABI(TMatrix6x6 & target, const TMatrix3x3 & aRb, const TMatrix6x6 & abi) {
    //target = makeForceTransform(aRb) * abi * makeVelocityTransform(TMatrix3x3(aRb.transpose())); return;
    target.block< 3, 3 >(0, 0) = aRb * abi.block< 3, 3 >(0, 0) * aRb.transpose();
    target.block< 3, 3 >(0, 3) = aRb * abi.block< 3, 3 >(0, 3) * aRb.transpose();
    target.block< 3, 3 >(3, 0) = target.block< 3, 3 >(0, 3).transpose(); // aRb * abi.block<3,3>(3,0) * aRb.transpose();
    target.block< 3, 3 >(3, 3) = aRb * abi.block< 3, 3 >(3, 3) * aRb.transpose();
}

void mbslib::transformABI(TMatrix6x6 & target, const TVector3 & arb, const TMatrix6x6 & abi) {

    //target = makeForceTransform(arb) * abi * makeVelocityTransform(TVector3(-arb)); return;
    TMatrix3x3 rx = makeCrossproductMatrix(arb);
    target.block< 3, 3 >(0, 0) = abi.block< 3, 3 >(0, 0) - abi.block< 3, 3 >(0, 3) * rx + rx * abi.block< 3, 3 >(3, 0) - rx * abi.block< 3, 3 >(3, 3) * rx;
    target.block< 3, 3 >(0, 3) = abi.block< 3, 3 >(0, 3) + rx * abi.block< 3, 3 >(3, 3);
    target.block< 3, 3 >(3, 0) = target.block< 3, 3 >(0, 3).transpose(); //abi.block<3,3>(3,0) - abi.block<3,3>(3,3) * rx;
    target.block< 3, 3 >(3, 3) = abi.block< 3, 3 >(3, 3);
}

void mbslib::transformABI(TMatrix6x6 & target, const TMatrix3x3 & aRb, const TVector3 & arb, const TMatrix6x6 & abi) {
    target = makeForceTransform(aRb, arb) * abi * makeVelocityTransform(TMatrix3x3(aRb.transpose()), TVector3(-arb));
}

TVector6 mbslib::spatialCross(const TVector6 & v1, const TVector6 & v2) {
    TVector6 c;
    TVector3 v2upper = v2.segment< 3 >(0);
    TVector3 v2lower = v2.segment< 3 >(3);
    c.segment< 3 >(0) = v1.segment< 3 >(0).cross(v2upper);
    c.segment< 3 >(3) = v1.segment< 3 >(3).cross(v2upper) + v1.segment< 3 >(0).cross(v2lower);
    return c;
}

TVector6 mbslib::spatialCrossStar(const TVector6 & v1, const TVector6 & v2) {
    TVector6 c;
    TVector3 v2upper = v2.segment< 3 >(0);
    TVector3 v2lower = v2.segment< 3 >(3);
    c.segment< 3 >(0) = v1.segment< 3 >(0).cross(v2upper) + v1.segment< 3 >(3).cross(v2lower);
    c.segment< 3 >(3) = v1.segment< 3 >(0).cross(v2lower);
    return c;
}

TVector3 mbslib::mirrorVector(const TVector3 & v, const TVector3 & normal) {
    return v - 2 * normal * v.dot(normal);
}

void condassign(mbslib::TScalar & a, mbslib::TScalar b, mbslib::TScalar c, mbslib::TScalar d) {
    a = (b > 0) ? c : d;
}
