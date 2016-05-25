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
 * \file mbslib/endpoint/force/ForceGeneratorSet.cpp
 * Definition of mbslib ::ForceGeneratorSet
 */

#include <mbslib/elements/force/ForceGeneratorSet.hpp>

using namespace mbslib;

void ForceGeneratorSet::add(ForceGenerator & fg) {
    generators.push_back(&fg);
}

void ForceGeneratorSet::applyForces() {
    for (ForceGenerator * fg : generators) {
        fg->applyForce();
    }
}

void ForceGeneratorSet::resetForces() {
    for (ForceGenerator * fg : generators) {
        fg->resetForce();
    }
}
const std::vector< ForceGenerator * > ForceGeneratorSet::getGenerators() const {
    return generators;
}

ForceGeneratorSet::~ForceGeneratorSet() {
}