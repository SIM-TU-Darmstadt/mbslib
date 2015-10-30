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
 * \file mbslib/utility/SimulationControl.cpp
 * Definition of mbslib::SimulationControl
 */
#include <mbslib/utility/SimulationControl.hpp>
#include <iostream>
using namespace mbslib;

SimulationControl::SimulationControl(MbsCompound & mbs, CollisionDetector & cd)
    : mbs(mbs)
    , cd(cd)
    , extF()
    , time(0) {
    mbs.doDirkin();
    cd.detectContacts();
}

SimulationControl::SimulationControl(MbsCompound & mbs, CollisionDetector & cd, ForceGeneratorSet & ef)
    : mbs(mbs)
    , cd(cd)
    , extF(&ef)
    , time(0) {
    mbs.doDirkin();
    cd.detectContacts();
}

void SimulationControl::doTimestep(TTime dt, TScalar maxPenetration) {
    time += dt;

    if (extF) {
        extF->resetForces();
        extF->applyForces();
    }
    mbs.doCrba(cd.getContacts());
    mbs.storeState();
    mbs.integrate(dt);
    mbs.doDirkin();
    cd.detectContacts();

    TScalar ddt = 0;
    for (std::vector< PointContact >::const_iterator it = cd.getContacts().begin(); it != cd.getContacts().end(); it++) {
        if (it->getDepth() > maxPenetration) {
            TScalar goback = it->getDepth() / it->getPenetrationVelocity();
            std::cout << "depth = " << it->getDepth() << " v = " << it->getPenetrationVelocity() << " goback = " << goback;
            if (goback > ddt) {
                ddt = goback;
            }
        }
    }
    //std::cout << " ddt = " << ddt << std::endl;
    if (ddt > 0) {
        std::cout << "going back" << std::endl;
        mbs.restoreState();
        mbs.doDirkin();
        cd.detectContacts();
        for (std::vector< PointContact >::const_iterator it = cd.getContacts().begin(); it != cd.getContacts().end(); it++) {
            std::cout << "depth = " << it->getDepth() << " v = " << it->getPenetrationVelocity();
        }
        std::cout << "#" << std::endl;

        if (extF) {
            extF->resetForces();
            extF->applyForces();
        }
        mbs.doCrba(cd.getContacts());
        mbs.integrate(dt - ddt);
        mbs.doDirkin();
        cd.detectContacts();
        for (std::vector< PointContact >::const_iterator it = cd.getContacts().begin(); it != cd.getContacts().end(); it++) {
            std::cout << "depth = " << it->getDepth() << " v = " << it->getPenetrationVelocity();
        }
        std::cout << "#" << std::endl;

        if (extF) {
            extF->resetForces();
            extF->applyForces();
        }
        mbs.doCrba(cd.getContacts());
        mbs.integrate(ddt);
        mbs.doDirkin();
        cd.detectContacts();
        for (std::vector< PointContact >::const_iterator it = cd.getContacts().begin(); it != cd.getContacts().end(); it++) {
            std::cout << "depth = " << it->getDepth() << " v = " << it->getPenetrationVelocity();
        }
        std::cout << "#" << std::endl;
    }
}
SimulationControl::~SimulationControl() {
}
