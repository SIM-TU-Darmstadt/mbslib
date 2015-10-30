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

#include <mbslib/elements/IIntegrate.hpp>

using namespace mbslib;

std::string IIntegrate::getStateVariableName(unsigned int) const {
    return "";
}
void IIntegrate::getLowerStateLimits(TVectorX & limits) const {
    int n = getNumberOfStateVariables();
    if (!n)
        return;
    limits.resize(n);
    for (int i = 0; i < n; i++) {
        limits(i) = -1e10;
    }
}
void IIntegrate::getUpperStateLimits(TVectorX & limits) const {
    int n = getNumberOfStateVariables();
    if (!n)
        return;
    limits.resize(n);
    for (int i = 0; i < n; i++) {
        limits(i) = 1e10;
    }
}
TVectorX IIntegrate::getLowerStateLimits() const {
    TVectorX limits(getNumberOfStateVariables());
    getLowerStateLimits(limits);
    return limits;
}
TVectorX IIntegrate::getUpperStateLimits() const {
    TVectorX limits(getNumberOfStateVariables());
    getUpperStateLimits(limits);
    return limits;
}

bool IIntegrate::getIsStateVariableUnconstrainedAngle(unsigned int i) const {
    return false;
}

void IIntegrate::getStateLimits(TVectorX & lower, TVectorX & upper) const {
    getLowerStateLimits(lower);
    getUpperStateLimits(upper);
}
IIntegrate::~IIntegrate() {
}
