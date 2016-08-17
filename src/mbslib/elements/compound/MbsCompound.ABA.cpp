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

void MbsCompound::doABA(bool dontRecalcExternalForces) {
    doABA(gravitationVector, dontRecalcExternalForces);
}

void MbsCompound::doABA(const TVector3 & g, bool dontRecalcExternalForces) {
    doPosition();
    doVelocity();
    if (!dontRecalcExternalForces) {
        forceGeneratorSet.resetForces();
        forceGeneratorSet.applyForces();
    }
    CoordinateFrame & basecof = base->getCoordinateFrame();
    //handling of gravitation: in case of a fixed base, we have to set a virtual
    //acceleration of -g to get the effects of gravitation at the joints
    //TODO: what do we have to do for a free base?
    if (!freeBase) {
        basecof.dotomega.setZero();
        basecof.dotv = basecof.R.transpose() * -g;
    }
    for (std::vector< MbsObject * >::iterator it = elements.begin(); it != elements.end(); ++it) {
        (**it).doABASweep1fwd();
    }
    for (std::vector< MbsObject * >::reverse_iterator it = elements.rbegin(); it != elements.rend(); ++it) {
        (**it).doABASweep2fwd();
    }
    for (std::vector< MbsObject * >::iterator it = elements.begin(); it != elements.end(); ++it) {
        (**it).doABASweep3fwd();
    }
    //handling of acceleration: after the algorithm, a fixed base will have acc = 0, the acc. of a free base will be
    //increased by g
    if (freeBase) {
        basecof.dotv += basecof.R.transpose() * g;
    } else {
        basecof.dotv.setZero();
    }
    doAcceleration();
}

void MbsCompound::doABAinv(bool dontRecalcExternalForces) {
    doABAinv(gravitationVector, dontRecalcExternalForces);
}

void MbsCompound::doABAinv(const TVector3 & g, bool dontRecalcExternalForces) {
    doPosition();
    doVelocity();
    if (!dontRecalcExternalForces) {
        forceGeneratorSet.resetForces();
        forceGeneratorSet.applyForces();
    }
    CoordinateFrame & basecof = base->getCoordinateFrame();
    //handling of gravitation: in case of a fixed base, we have to set a virtual
    //acceleration of -g to get the effects of gravitation at the joints
    //TODO: what do we have to do for a free base?
    if (!freeBase) {
        basecof.dotomega.setZero();
        basecof.dotv = basecof.R.transpose() * -g;
    }
    for (std::vector< MbsObject * >::iterator it = elements.begin(); it != elements.end(); ++it) {
        (**it).doABASweep1inv();
    }
    for (std::vector< MbsObject * >::reverse_iterator it = elements.rbegin(); it != elements.rend(); ++it) {
        (**it).doABASweep2inv();
    }
    for (std::vector< MbsObject * >::iterator it = elements.begin(); it != elements.end(); ++it) {
        (**it).doABASweep3inv();
    }
    //handling of acceleration: after the algorithm, a fixed base will have acc = 0, the acc. of a free base will be
    //increased by g
    if (freeBase) {
        basecof.dotv += basecof.R.transpose() * g;
    } else {
        basecof.dotv.setZero();
    }
    doAcceleration();
}

void MbsCompound::doABAhyb(const std::vector< bool > & doDirectDyn, bool dontRecalcExternalForces) {
    doABAhyb(gravitationVector, doDirectDyn, dontRecalcExternalForces);
}

void MbsCompound::doABAhyb(const TVector3 & g, const std::vector< bool > & doDirectDyn, bool dontRecalcExternalForces) {
    doPosition();
    doVelocity();
    if (!dontRecalcExternalForces) {
        forceGeneratorSet.resetForces();
        forceGeneratorSet.applyForces();
    }
    CoordinateFrame & basecof = base->getCoordinateFrame();
    //handling of gravitation: in case of a fixed base, we have to set a virtual
    //acceleration of -g to get the effects of gravitation at the joints
    //TODO: what do we have to do for a free base?
    if (!freeBase) {
        basecof.dotomega.setZero();
        basecof.dotv = basecof.R.transpose() * -g;
    }
    for (std::vector< MbsObject * >::iterator it = elements.begin(); it != elements.end(); ++it) {
        (**it).doABASweep1hyb(doDirectDyn);
    }
    for (std::vector< MbsObject * >::reverse_iterator it = elements.rbegin(); it != elements.rend(); ++it) {
        (**it).doABASweep2hyb(doDirectDyn);
    }
    for (std::vector< MbsObject * >::iterator it = elements.begin(); it != elements.end(); ++it) {
        (**it).doABASweep3hyb(doDirectDyn);
    }
    //handling of acceleration: after the algorithm, a fixed base will have acc = 0, the acc. of a free base will be
    //increased by g
    if (freeBase) {
        basecof.dotv += basecof.R.transpose() * g;
    } else {
        basecof.dotv.setZero();
    }
    doAcceleration();
}
