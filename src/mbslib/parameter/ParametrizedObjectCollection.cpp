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
 * \file mbslib/parameter/ParametrizedObjectCollection.cpp
 * Definition of mbslib::ParametrizedObjectCollection
 */

#include <iostream>
#include <mbslib/parameter/ParameterAdapter.hpp>
#include <mbslib/parameter/ParametrizedObjectCollection.hpp>

using namespace mbslib;

ParametrizedObject * ParametrizedObjectCollection::getParametrizedObject(const std::string & oname) {
    for (size_t i = 0; i < getParametrizedObjectCount(); i++) {
        ParametrizedObject & po = getParametrizedObject(i);
        if (po.getName() == oname) {
            return &po;
        }
    }
    return NULL;
}

const ParametrizedObject * ParametrizedObjectCollection::getParametrizedObject(const std::string & oname) const {
    for (size_t i = 0; i < getParametrizedObjectCount(); i++) {
        const ParametrizedObject & po = getParametrizedObject(i);
        if (po.getName() == oname) {
            return &po;
        }
    }
    return NULL;
}

ParameterAdapter ParametrizedObjectCollection::getParameterAdapter(const std::string & oname, const std::string & pname) {
    for (size_t i = 0; i < getParametrizedObjectCount(); i++) {
        ParametrizedObject & po = getParametrizedObject(i);
        if (po.getName() == oname) {
            return po.getParameterAdapter(pname);
        }
    }
    std::cout << "ParametrizedObjectCollection::getParameterAdapter -- unknown object " << oname << std::endl;
    return ParameterAdapter();
}
ParametrizedObjectCollection::~ParametrizedObjectCollection() {
}
