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
 * \file mbslib/elements/spring/Spring.cpp
 * Definition of mbslib ::Spring
 */

#include <mbslib/elements/spring/Spring.hpp>

using namespace mbslib;

Spring::Spring(const SpringModel & model, const std::string & name)
    : ParametrizedObject(name)
    , model(model.clone())
    , f(0)
    , Lm(0)
    , Vm(0) {
}

Spring::~Spring() {
    delete model;
}

IIntegrate * Spring::getIntegrator() {
    // The only way to make a spring stateful is to have a model which is stateful
    // and thus inherits IIntegrate. So we can get the integrator by dynamical
    // casting of the model.
    return dynamic_cast< IIntegrate * >(model);
}

TScalar Spring::getSpringForce() const {
    return f;
}

TScalar Spring::getSpringLength() const {
    return Lm;
} // added by MH

TScalar Spring::getSpringVelocity() const {
    return Vm;
} // added by MH

/**
 * \brief Get model of this Spring.
 *
 * \return  The spring model.
 */
SpringModel & Spring::getModel() {
    return *model;
}

/**
 * \brief Get model of this spring.
 *
 * \return  The spring model.
 */
const SpringModel & Spring::getModel() const {
    return *model;
}

/**
 * \brief Get name of this Spring.
 *
 * \return  The name.
 */
const std::string & Spring::getName() const {
    return ParametrizedObject::getName();
}
