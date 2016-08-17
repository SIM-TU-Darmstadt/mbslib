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

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <mbslib/elements/MbsCompound.hpp>
#include <mbslib/elements/base/FixedBase.hpp>
#include <mbslib/elements/base/FreeBase.hpp>
#include <mbslib/utility/internalTests.hpp>
#include <mbslib/utility/mathtools.hpp>

using namespace mbslib;

void MbsCompound::doRne(bool dontRecalcExternalForces) {
    doRne(gravitationVector, dontRecalcExternalForces);
}

void MbsCompound::doRne(const TVector3 & g, bool dontRecalcExternalForces) {
    //store initial acceleration of base
    TVector3 dotvbase = base->getCoordinateFrame().dotv;

    //set acceleration of base to compensate for gravitation
    base->getCoordinateFrame().dotv = base->getCoordinateFrame().R.transpose() * -g;

    //calculate outward sweep of rne
    doDirkin();

    //calculate forces
    if (!dontRecalcExternalForces) {
        forceGeneratorSet.resetForces();
        forceGeneratorSet.applyForces();
    }

    //calculate inward sweep of rne
    doRneInward(true);

    //restore initial acceleration of base
    base->getCoordinateFrame().dotv = dotvbase;
    doAcceleration();
}
