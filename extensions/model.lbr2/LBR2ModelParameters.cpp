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

#include <cmath>
#include <limits.h>
#include <limits>
#include <math.h>
#include <model.lbr2/LBR2ModelParameters.h>

using namespace mbslib;

namespace mbslib {
LBR2ModelParameters getLBR2ModelParameters() {
    LBR2ModelParameters p;

    std::vector< LBR2Arm > & arm = p.arm;
    TVector3 & g = p.g;

    p.arm.resize(7);

    TVectorX a(7);
    a << 0.0, 0.25, 0.26, 0.0, 0.0000, 0.0, 0.0;
    TVectorX d(7);
    d << 0.213, 0.00, 0.00, 0.0, 0.3385, 0.0, 0.0785;
    TVectorX alpha(7);
    alpha << -M_PI / 2, M_PI / 2, -M_PI / 2, M_PI / 2, -M_PI / 2, M_PI / 2, 0.0;
    TVectorX theta(7);
    theta << 0.0, -M_PI / 2, 0.0, M_PI / 2, 0.0, 0.0, 0.0;

    for (size_t i = 0; i < 7; ++i) {
        p.arm[i].jointType = JointType::Revolute;
        p.arm[i].a = a(i);
        p.arm[i].d = d(i);
        p.arm[i].alpha = alpha(i);
        p.arm[i].theta = theta(i);
    }

    g = TVector3(0, 0, -9.81);

    arm[0].axis = TVector3::UnitZ();
    arm[0].l = TVector3(0.0, 0.0, 0.213);
    arm[0].m = 1.7780;
    arm[0].com = TVector3(0.0, 0.0, 0.0);
    arm[0].I << 0.0, 0.0, 0.0,
        0.0, 0.4996e-2, 0.0,
        0.0, 0.0, 0.0;
    //arm[0].I << 0.0, 0.0, 0.0,
    //            0.0, 0.0, 0.0,
    //            0.0, 0.0, 0.4996e-2;

    arm[1].axis = TVector3::UnitZ();
    arm[1].l = TVector3(0.0, -0.25, 0.0);
    arm[1].m = 5.0249;
    arm[1].com = TVector3(0.1042 - 0.25, 0.0008858, -0.0023359);
    arm[1].I << 0.8870e-2, 0.0, 0.2528e-2,
        0.0, 0.1234, 0.0,
        0.2528e-2, 0.0, 0.1235;
    //arm[1].I << 0.8870e-2, 0.2528e-2, 0.0,
    //            0.2528e-2, 0.1235, 0.0,
    //            0.0, 0.0, 0.1234;

    arm[2].axis = TVector3::UnitZ();
    arm[2].l = TVector3(0.26, 0.0, 0.0);
    arm[2].m = 1.7877;
    arm[2].com = TVector3(0.1111 - 0.26, -0.005409, -0.001836);
    arm[2].I << 0.6763e-2, 0.0, 0.1746e-2,
        0.0, 0.46153e-1, 0.0,
        0.1746e-2, 0.0, 0.4613e-1;
    //arm[2].I << 0.6763e-2, 0.1746e-2, 0.0,
    //            0.1746e-2, 0.4613e-1, 0.0,
    //            0.0, 0.0, 0.46153e-1;

    arm[3].axis = TVector3::UnitZ();
    arm[3].l = TVector3(0.0, 0.0, 0.0);
    arm[3].m = 3.0685;
    arm[3].com = TVector3(0.0017297, -0.002953, 0.01059);
    arm[3].I << 0.2058e-1, 0.0, 0.0,
        0.0, 0.2006e-1, 0.0,
        0.0, 0.0, 0.4404e-2;
    //arm[3].I << 0.2058e-1, 0.0, 0.0,
    //        0.0, 0.4404e-2, 0.0,
    //        0.0, 0.0, 0.2006e-1;

    arm[4].axis = TVector3::UnitZ();
    arm[4].l = TVector3(0.0, 0.0, 0.3385);
    arm[4].m = 1.1915;
    arm[4].com = TVector3(-0.00001299, -0.11986, 0.011362);
    arm[4].I << 0.3077e-1, 0.0, 0.0,
        0.0, 0.328e-2, 0.0,
        0.0, 0.0, 0.2924e-1;
    //arm[4].I << 0.3077e-1, 0.0, 0.0,
    //            0.0, 0.2924e-1, 0.0,
    //            0.0, 0.0, 0.328e-2;

    arm[5].axis = TVector3::UnitZ();
    arm[5].l = TVector3(0.0, 0.0, 0.0);
    arm[5].m = 2.8785;
    arm[5].com = TVector3(0.0018438, -0.00459189, 0.01130);
    arm[5].I << 0.20423e-1, 0.0, 0.0,
        0.0, 0.1997e-1, 0.0,
        0.0, 0.0, 0.4233e-2;
    //arm[5].I << 0.20423e-1, 0.0, 0.0,
    //            0.0, 0.4233e-2, 0.0,
    //            0.0, 0.0, 0.1997e-1;

    arm[6].axis = TVector3::UnitZ();
    arm[6].l = TVector3(0.0, 0.0, 0.0785);
    arm[6].m = 0.5721;
    arm[6].com = TVector3(-0.00002610, 0.0001501, 0.058534);
    arm[6].I << 0.2657e-2, 0.0, 0.0,
        0.0, 0.268e-2, 0.0,
        0.0, 0.0, 0.8615e-3;

    return p;
}
}
