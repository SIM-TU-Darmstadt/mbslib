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

#ifndef __MBSLIB_LBR2MODELPARAMETERS_HPP__
#define __MBSLIB_LBR2MODELPARAMETERS_HPP__

#include <model.lbr2/LBR2Arm.h>
#include <vector>

namespace mbslib {

struct LBR2ModelParameters {
    std::vector< LBR2Arm > arm;
    TVector3 g;
};

LBR2ModelParameters getLBR2ModelParameters();
}

#endif //__MBSLIB_LBR2MODELPARAMETERS_HPP__
