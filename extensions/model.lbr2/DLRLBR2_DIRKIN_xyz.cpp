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

#include <model.lbr2/dlrlbr2.hpp>

mbslib::TVector3 DLRLBR2_DIRKIN_xyz(const mbslib::TMatrixX & q) {
    mbslib::TVector3 r;

    const double q1 = q(0);
    const double q2 = q(1);
    const double q3 = q(2);
    const double q4 = q(3);
    const double q5 = q(4);
    const double q6 = q(5);
    const double q7 = q(6);

    const double t2 = sin(q1);
    const double t3 = sin(q3);
    const double t4 = cos(q1);
    const double t5 = 3.141592653589793 * (1.0 / 2.0);
    const double t6 = q4 + t5;
    const double t7 = q2 - t5;
    const double t8 = cos(t6);
    const double t9 = t2 * t3;
    const double t10 = cos(q3);
    const double t11 = cos(t7);
    const double t15 = t4 * t10 * t11;
    const double t12 = t9 - t15;
    const double t13 = sin(t7);
    const double t14 = sin(t6);
    const double t16 = cos(q6);
    const double t17 = t3 * t4;
    const double t18 = t2 * t10 * t11;
    const double t19 = t17 + t18;
    const double t20 = sin(q6);
    const double t21 = cos(q5);
    const double t22 = sin(q5);
    r(0) = t16 * (t12 * t14 - t4 * t8 * t13) * (-7.85E-2) - t2 * t3 * (1.3E1 / 5.0E1) - t12 * t14 * 3.385E-1 - t20 * (t22 * (t2 * t10 + t3 * t4 * t11) + t21 * (t8 * t12 + t4 * t13 * t14)) * 7.85E-2 + t4 * cos(q2 - 3.141592653589793 * (1.0 / 2.0)) * (1.0 / 4.0) + t4 * t8 * t13 * 3.385E-1 + t4 * t10 * t11 * (1.3E1 / 5.0E1);
    r(1) = t16 * (t14 * t19 + t2 * t8 * t13) * 7.85E-2 + t3 * t4 * (1.3E1 / 5.0E1) + t2 * t11 * (1.0 / 4.0) + t14 * t19 * 3.385E-1 + t20 * (t22 * (t4 * t10 - t2 * t3 * t11) + t21 * (t8 * t19 - t2 * t13 * t14)) * 7.85E-2 + t2 * t8 * t13 * 3.385E-1 + t2 * t10 * t11 * (1.3E1 / 5.0E1);
    r(2) = t13 * (-1.0 / 4.0) - t20 * (t21 * (t11 * t14 + t8 * t10 * t13) - t3 * t13 * t22) * 7.85E-2 + t16 * (t8 * t11 - t10 * t13 * t14) * 7.85E-2 + t8 * t11 * 3.385E-1 - t10 * t13 * (1.3E1 / 5.0E1) - t10 * t13 * t14 * 3.385E-1 + 2.13E2 / 1.0E3;

    return r;
}
