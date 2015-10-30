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

#include <mbslib/parameter/ParameterAdapter.hpp>

using namespace mbslib;

/**
* \brief Default constructor.
*/
ParameterAdapter::ParameterAdapter()
    : object(nullptr)
    , paramid(0) {
}

/**
* \brief Constructor. \todo kommentieren
*
* \param o The parameterized object.
* \param p The p. \todo kommentieren
*/
ParameterAdapter::ParameterAdapter(ParametrizedObject & o, size_t p)
    : object(&o)
    , paramid(p) {
}

/**
* \brief Check for validity of ParameterAdapter.
*  
*  If this return false, the ParameterAdapter does not access an existing parameter.
*
* \return  true if valid, false if not.
*/
bool ParameterAdapter::isValid() const {
    return (object != NULL);
}

/**
* \brief Assignment of a value to the parameter.
*
* \param value The value.
*
* \return  A shallow copy of this object.
*/
TScalar ParameterAdapter::operator=(TScalar value) {
    assert(object);
    object->setParameter(paramid, value);
    return value;
}

/**
* \brief Conversion operator to scalar type.
*/
ParameterAdapter::operator TScalar() {
    assert(object);
    return object->getParameter(paramid);
}

std::string ParameterAdapter::getName() const {
    if (isValid()) {
        return object->getParameterName(paramid);
    }
    return "";
}
size_t ParameterAdapter::getID() const {
    return paramid;
}
ParameterAdapter::~ParameterAdapter() {
}
