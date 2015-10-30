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
 * \file mbslib/ForceControlledMuscleModel.cpp
 * Definition of mbslib ::ForceControlledMuscleModel
 */
#include <mbslib/elements/muscle/model/ForceControlledMuscleModel.hpp>

using namespace mbslib;

ForceControlledMuscleModel::ForceControlledMuscleModel()
    : f(0) {
}

void ForceControlledMuscleModel::setControlValue(TScalar c, unsigned int valueId) {
    assert(valueId == 0);
    f = c;
};

TScalar ForceControlledMuscleModel::getControlValue(unsigned int valueId) const {
    assert(valueId == 0);
    return f;
};

const std::string & ForceControlledMuscleModel::getControlValueName(unsigned int valueId) const {
    assert(valueId == 0);
    static const std::string controlName = "force";
    return controlName;
}

TScalar ForceControlledMuscleModel::calculateForce(TScalar l, TScalar dl) const {
    if (!getDeriveMode()) {
        return f;
    } else {
        // as for this simple muscle  force == control value, the derivative is globally 1
        return 1;
    }
}

TScalar ForceControlledMuscleModel::calculateDForceDLength(TScalar l, TScalar dl) const {
    return TScalar(0);
}

TScalar ForceControlledMuscleModel::calculateDForceDVelocity(TScalar l, TScalar dl) const {
    return TScalar(0);
}

SpringModel * ForceControlledMuscleModel::clone() const {
    return new ForceControlledMuscleModel();
}

unsigned int ForceControlledMuscleModel::getNumberOfControlValues(void) const {
    return 1;
}

ForceControlledMuscleModel::~ForceControlledMuscleModel() {
}
