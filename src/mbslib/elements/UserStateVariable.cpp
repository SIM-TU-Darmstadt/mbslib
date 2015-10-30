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

#include <mbslib/elements/UserStateVariable.hpp>

using namespace mbslib;

UserStateVariable::UserStateVariable(size_t dimension)
    : _dimension(dimension)
    , _state(dimension)
    , _dState(dimension)
    , _storedState(dimension)
    , _storedDState(dimension)
    , _maxValue(dimension)
    , _minValue(dimension) {

    for (size_t i = 0; i < dimension; i++) {
        _unconstrainedAngle.push_back(false);
        _variableNames.push_back("");
        _maxValue(i) = 1e10;
        _minValue(i) = -1e10;
    }
}
UserStateVariable::~UserStateVariable() {
}

/**
* \brief Integrate one timestep.
*
* \param dt  The timeintervall to integrate over
*/
void UserStateVariable::integrate(TTime dt) {
    // simple explicit euler
    setState(getState() + getDStateDt() * dt);
}

/**
* \brief Store current state.
*/
void UserStateVariable::storeState() {
    _storedState = _state;
    _storedDState = _dState;
}

/**
* \brief Go back to last stored state.
*/
void UserStateVariable::restoreState() {
    _state = _storedState;
    _dState = _storedDState;
}

/**
* \brief Get number of state variables.
*
* \return  The number of state variables.
*/
size_t UserStateVariable::getNumberOfStateVariables() const {
    return _dimension;
}

/**
* \brief Get state.
*
* \return  The state vector.
*/
TVectorX UserStateVariable::getState() const {
    return _state;
}

/**
* \brief indicates whether the variable at i is an unconstrained angle
*
*/
bool UserStateVariable::getIsStateVariableUnconstrainedAngle(unsigned int i) const {
    return _unconstrainedAngle[i];
}
void UserStateVariable::setIsStateVariableUnconstrainedAngle(unsigned int i, bool value) {
    _unconstrainedAngle[i] = value;
}

/**
* \brief Get first derivative of state wrt. time.
*
* \return  First derivative of state wrt. time.
*/
TVectorX UserStateVariable::getDStateDt() const {
    return _dState;
}
// set the derived state
void UserStateVariable::setDStateDt(const TVectorX & dState) {
    _dState = dState;
}
// sets a entry in derived state
void UserStateVariable::setDStateDt(int i, TScalar value) {
    _dState(i) = value;
}
// sets the first value in derived state
void UserStateVariable::setDStateDt(TScalar value) {
    setDStateDt(0, value);
}

/**
* \brief Set state.
*
* \param state The state.
*/
void UserStateVariable::setState(const TVectorX & state) {
    _state = state;
}
void UserStateVariable::setState(int i, TScalar value) {
    _state(i) = value;
}
void UserStateVariable::setState(TScalar value) {
    setState(0, value);
}

std::string UserStateVariable::getStateVariableName(unsigned int i) const {
    return _variableNames[i];
}
void UserStateVariable::setStateVariableName(unsigned int i, const std::string & value) {
    _variableNames[i] = value;
}

void UserStateVariable::getLowerStateLimits(TVectorX & limits) const {
    limits = _minValue;
}
void UserStateVariable::getUpperStateLimits(TVectorX & limits) const {
    limits = _maxValue;
}

void UserStateVariable::getStateLimits(TVectorX & lower, TVectorX & upper) const {
    getLowerStateLimits(lower);
    getUpperStateLimits(upper);
}
