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
 * \file src/example/exp_02_6dof.cpp
 * 
 */
#include <mbslib/mbslib.hpp>

#include <Eigen/Geometry>

#include <iostream>

#include <math.h>
using namespace mbslib;
void addDhLink(mbslib::MbsCompoundWithBuilder & mbs, bool revolute, TScalar theta, TScalar d, TScalar a, TScalar alpha, const TVector3 & com, TScalar m, const TVector3 & Idiag) {
    //a = 1; d= 0;
    if (revolute) {
        //mbs.addRevoluteJoint(TVector3::UnitZ(), theta);
        mbs.addRevoluteJointZ(theta);
        TMatrix3x3 rotX;
        rotX = Eigen::AngleAxis< TScalar >(alpha, TVector3::UnitX());
        mbs.addRigidLink(TVector3(a, 0, d), rotX * com, m * 0.1, rotX * Idiag.asDiagonal() * rotX.transpose());
        mbs.addFixedRotation(rotX);
    }
}

TScalar toRad(TScalar deg) {
    return M_PI * deg / 180.;
}

int main(void) {
    using namespace mbslib;

    MbsCompoundWithBuilder mbs;

    //Create Kuka-Robot from example
    mbs.addFixedBase();
    addDhLink(mbs, true, 0, 0.865, 0.41, toRad(-90), TVector3(-0.314, 0.259, -0.012), 686.78, TVector3(72.6632, 95.1493, 84.4584));
    addDhLink(mbs, true, toRad(-90), 0, 1, 0, TVector3(-0.737, -0.005, -0.188), 309.59, TVector3(10.6252, 49.5171, 45.7713));
    addDhLink(mbs, true, 0, 0, 0, toRad(90), TVector3(0.041, -0.109, -0.035), 100.11, TVector3(4.808, 4.6959, 3.5356));
    addDhLink(mbs, true, 0, -1, 0, toRad(-90), TVector3(0, -0.65, 0), 145, TVector3(28.4107, 1.8134, 28.0755));
    addDhLink(mbs, true, 0, 0, 0, toRad(90), TVector3(0, -0.0097, 0.1), 16.52, TVector3(0.1185, 0.1037, 0.0498));
    addDhLink(mbs, true, 0, -0.21, 0, toRad(180), TVector3(0, 0, 0), 5, TVector3(0.1185, 0.1037, 0.0498));

    addDhLink(mbs, true, 0, 0.865, 0.41, toRad(-90), TVector3(-0.314, 0.259, -0.012), 686.78, TVector3(72.6632, 95.1493, 84.4584));
    addDhLink(mbs, true, toRad(-90), 0, 1, 0, TVector3(-0.737, -0.005, -0.188), 309.59, TVector3(10.6252, 49.5171, 45.7713));
    addDhLink(mbs, true, 0, 0, 0, toRad(90), TVector3(0.041, -0.109, -0.035), 100.11, TVector3(4.808, 4.6959, 3.5356));
    addDhLink(mbs, true, 0, -1, 0, toRad(-90), TVector3(0, -0.65, 0), 145, TVector3(28.4107, 1.8134, 28.0755));
    addDhLink(mbs, true, 0, 0, 0, toRad(90), TVector3(0, -0.0097, 0.1), 16.52, TVector3(0.1185, 0.1037, 0.0498));
    addDhLink(mbs, true, 0, -0.21, 0, toRad(180), TVector3(0, 0, 0), 5, TVector3(0.1185, 0.1037, 0.0498));

    addDhLink(mbs, true, 0, 0.865, 0.41, toRad(-90), TVector3(-0.314, 0.259, -0.012), 686.78, TVector3(72.6632, 95.1493, 84.4584));
    addDhLink(mbs, true, toRad(-90), 0, 1, 0, TVector3(-0.737, -0.005, -0.188), 309.59, TVector3(10.6252, 49.5171, 45.7713));
    addDhLink(mbs, true, 0, 0, 0, toRad(90), TVector3(0.041, -0.109, -0.035), 100.11, TVector3(4.808, 4.6959, 3.5356));
    addDhLink(mbs, true, 0, -1, 0, toRad(-90), TVector3(0, -0.65, 0), 145, TVector3(28.4107, 1.8134, 28.0755));
    addDhLink(mbs, true, 0, 0, 0, toRad(90), TVector3(0, -0.0097, 0.1), 16.52, TVector3(0.1185, 0.1037, 0.0498));
    addDhLink(mbs, true, 0, -0.21, 0, toRad(180), TVector3(0, 0, 0), 5, TVector3(0.1185, 0.1037, 0.0498));

    addDhLink(mbs, true, 0, 0.865, 0.41, toRad(-90), TVector3(-0.314, 0.259, -0.012), 686.78, TVector3(72.6632, 95.1493, 84.4584));
    addDhLink(mbs, true, toRad(-90), 0, 1, 0, TVector3(-0.737, -0.005, -0.188), 309.59, TVector3(10.6252, 49.5171, 45.7713));
    addDhLink(mbs, true, 0, 0, 0, toRad(90), TVector3(0.041, -0.109, -0.035), 100.11, TVector3(4.808, 4.6959, 3.5356));
    addDhLink(mbs, true, 0, -1, 0, toRad(-90), TVector3(0, -0.65, 0), 145, TVector3(28.4107, 1.8134, 28.0755));
    addDhLink(mbs, true, 0, 0, 0, toRad(90), TVector3(0, -0.0097, 0.1), 16.52, TVector3(0.1185, 0.1037, 0.0498));
    addDhLink(mbs, true, 0, -0.21, 0, toRad(180), TVector3(0, 0, 0), 5, TVector3(0.1185, 0.1037, 0.0498));

    addDhLink(mbs, true, 0, 0.865, 0.41, toRad(-90), TVector3(-0.314, 0.259, -0.012), 686.78, TVector3(72.6632, 95.1493, 84.4584));
    addDhLink(mbs, true, toRad(-90), 0, 1, 0, TVector3(-0.737, -0.005, -0.188), 309.59, TVector3(10.6252, 49.5171, 45.7713));
    addDhLink(mbs, true, 0, 0, 0, toRad(90), TVector3(0.041, -0.109, -0.035), 100.11, TVector3(4.808, 4.6959, 3.5356));
    addDhLink(mbs, true, 0, -1, 0, toRad(-90), TVector3(0, -0.65, 0), 145, TVector3(28.4107, 1.8134, 28.0755));
    addDhLink(mbs, true, 0, 0, 0, toRad(90), TVector3(0, -0.0097, 0.1), 16.52, TVector3(0.1185, 0.1037, 0.0498));
    addDhLink(mbs, true, 0, -0.21, 0, toRad(180), TVector3(0, 0, 0), 5, TVector3(0.1185, 0.1037, 0.0498));

    addDhLink(mbs, true, 0, 0.865, 0.41, toRad(-90), TVector3(-0.314, 0.259, -0.012), 686.78, TVector3(72.6632, 95.1493, 84.4584));
    addDhLink(mbs, true, toRad(-90), 0, 1, 0, TVector3(-0.737, -0.005, -0.188), 309.59, TVector3(10.6252, 49.5171, 45.7713));
    addDhLink(mbs, true, 0, 0, 0, toRad(90), TVector3(0.041, -0.109, -0.035), 100.11, TVector3(4.808, 4.6959, 3.5356));
    addDhLink(mbs, true, 0, -1, 0, toRad(-90), TVector3(0, -0.65, 0), 145, TVector3(28.4107, 1.8134, 28.0755));
    addDhLink(mbs, true, 0, 0, 0, toRad(90), TVector3(0, -0.0097, 0.1), 16.52, TVector3(0.1185, 0.1037, 0.0498));
    addDhLink(mbs, true, 0, -0.21, 0, toRad(180), TVector3(0, 0, 0), 5, TVector3(0.1185, 0.1037, 0.0498));

    //todo: add tool...

    Endpoint * tcp = mbs.addEndpoint();

    mbs.doDirkin();

    /*
  TVectorX q;
  q.setZero(6);
  for(TScalar q1 = -1; q1 < 1; q1 += 0.5){
  for(TScalar q2 = -1; q2 < 1; q2 += 0.5){
  for(TScalar q3 = -1; q3 < 1; q3 += 0.5){
  for(TScalar q4 = -1; q4 < 1; q4 += 0.5){
  for(TScalar q5 = -1; q5 < 1; q5 += 0.5){
  for(TScalar q6 = -1; q6 < 1; q6 += 0.5){
    q(0) = q1;
    q(1) = q2;
    q(2) = q3;
    q(3) = q4;
    q(4) = q5;
    q(5) = q6;
    mbs.setJointPosition(q);
    std::cout << q << " " <<  (( mbs.calculateMassMatrix() - mbs.calculateMassMatrix2() ).cwise().abs()) << std::endl;// << mbs.calculateMassMatrix() - mbs.calculateMassMatrix2() << std::endl << std::endl;
  }
  }
  }
  }
  }
  }
  */

    for (int i = 0; i < 100000; i++) {
        mbs.doCrba();
    }

    return 0;
}