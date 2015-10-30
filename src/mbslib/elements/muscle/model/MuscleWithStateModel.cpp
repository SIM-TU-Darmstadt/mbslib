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
#include <mbslib/elements/muscle/model/MuscleWithStateModel.hpp>

MuscleWithStateModel::MuscleWithStateModel()
    : deriveMode(false)
    , deriveByValueId(0)
    , force(0)
    , controlValue(0)
    , storedForce(0)
    , stored_dForce_dT(0) {
}

void MuscleWithStateModel::setDeriveMode(bool dm, unsigned int valueId) {
    // there is only one control variable
    // we assert this, so that in Debug-mode erroneous use is caught
    assert(valueId == 0);
    // set derive mode
    deriveMode = dm;
    deriveByValueId = valueId;
}

bool MuscleWithStateModel::getDeriveMode() const {
    return deriveMode;
}

void MuscleWithStateModel::setControlValue(TScalar c, unsigned int valueId) {
    assert(valueId == 0);
    controlValue = c;
}

TScalar MuscleWithStateModel::getControlValue(unsigned int valueId) const {
    assert(valueId == 0);
    return controlValue;
}

unsigned int MuscleWithStateModel::getNumberOfControlValues(void) const {
    return 1;
}

const std::string & MuscleWithStateModel::getControlValueName(unsigned int valueId) const {
    static const std::string cv = "desiredForce";
    static const std::string err = "illegalControlValueId";
    switch (valueId) {
    case 0:
        return cv;
        break;
    default:
        return err;
        break;
    }
}

TScalar MuscleWithStateModel::calculateForce(TScalar l, TScalar dl) const {
    // actually the force is not calculate here, as the force is directly given as the
    // state variable of this muscle.

    // the actual differential equation is quite simple: df/dt = desiredForce - currentForce

    // depending on the derive mode, we return either the force or the derivative wrt. the selected control value.
    if (!deriveMode) {
        return force;
    } else {
        assert(deriveByValueId == 0);
        return 1;
    }
}

TScalar MuscleWithStateModel::calculateDForceDLength(TScalar l, TScalar dl) const {
    // the force of this muscle is independent of the current geometry...
    return 0;
}

TScalar MuscleWithStateModel::calculateDForceDVelocity(TScalar l, TScalar dl) const {
    // the force of this muscle is independent of the current geometry...
    return 0;
}

SpringModel * MuscleWithStateModel::clone() const {
    // The clone method must return a clone of the model with all its parameters.
    return new MuscleWithStateModel();
}

void MuscleWithStateModel::integrate(TTime dt) {
    // A simple Euler-integrator.
    force = force + dt * getDStateDt()(0);
}

void MuscleWithStateModel::storeState() {
    // The state is stored in case we need to go back in time.
    // Note that this methodology is quite simple, and for more complex scenarios (e.g. when keeping
    // track of several states) this must be done by the user outside of the mbslib!
    storedForce = force;
}

void MuscleWithStateModel::restoreState() {
    // Restore state . See comments in method storeState()
    force = storedForce;
}

size_t MuscleWithStateModel::getNumberOfStateVariables() const {
    // This model has one state variable.
    return 1;
}

TVectorX MuscleWithStateModel::getState() const {
    // Create a dynamic vector of size one ...
    TVectorX s(1);
    // ... fill it with the state variable(s) ...
    s(0) = force;
    // ... and return it:
    return s;
}

TVectorX MuscleWithStateModel::getDStateDt() const {
    // Create a dynamic vector of size one ...
    TVectorX s(1);
    // ... fill it with the derivatives of the state variable(s) which is given by the muscles deq....
    s(0) = controlValue - force;
    // ... and return it:
    return s;
}

void MuscleWithStateModel::setState(const TVectorX & state) {
    // We use an assert to check in Debug-mode whether the new state vector is of proper size.
    assert(state.size() == 1);
    // Extract state variables from state vector.
    force = state(0); // changed to minus because muscle should contract
}
MuscleWithStateModel::~MuscleWithStateModel() {
}