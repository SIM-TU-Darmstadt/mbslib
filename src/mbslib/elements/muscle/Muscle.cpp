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
 * \file mbslib/elements/muscle/Muscle.cpp
 * Definition of mbslib ::Muscle
 */

#include <mbslib/elements/muscle/Muscle.hpp>

using namespace mbslib;

Muscle::Muscle(const MuscleModel & model, Endpoint & end1, Endpoint & end2, const std::string & name)
    : Spring3D(model, end1, end2, name) {
}

Muscle::Muscle(const MuscleModel & model, std::vector< Endpoint * > points, const std::string & name)
    : Spring3D(model, points, name) {
}

Muscle::Muscle(const MuscleModel & model, const std::string & name)
    : Spring3D(model, name) {
}

void Muscle::setDeriveMode(bool dm, unsigned int valueId) {
    // added by MH  for old Stelzer
    // deriveMode = dm;
    // deriveBy = valueId;

    static_cast< MuscleModel & >(getModel()).setDeriveMode(dm, valueId);
}

bool Muscle::getDeriveMode() const {
    return static_cast< const MuscleModel & >(getModel()).getDeriveMode();
}

void Muscle::setControlValue(TScalar c, unsigned int valueId) {
    static_cast< MuscleModel & >(getModel()).setControlValue(c, valueId);
}

TScalar Muscle::getControlValue(unsigned int valueId) const {
    return static_cast< const MuscleModel & >(getModel()).getControlValue(valueId);
}

unsigned int Muscle::getNumberOfControlValues(void) const {
    return static_cast< const MuscleModel & >(getModel()).getNumberOfControlValues();
}

const std::string & Muscle::getControlValueName(unsigned int valueId) const {
    static const std::string cv = "Activation";
    return static_cast< const MuscleModel & >(getModel()).getControlValueName(valueId);
}
Muscle::~Muscle() {
}
