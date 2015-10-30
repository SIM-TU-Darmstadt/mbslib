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
 * \file mbslib/elements/spring/Spring.hpp
 * Declaration of mbslib ::Spring
 */
#ifndef __MBSLIB_SPRING_HPP__
#define __MBSLIB_SPRING_HPP__

#include <mbslib/elements/force/ForceGenerator.hpp>
#include <mbslib/parameter/ParametrizedObject.hpp>

#include <mbslib/elements/spring/SpringModel.hpp>

namespace mbslib {

/**
 * \brief Spring. \todo kommentieren
 */
class Spring : public virtual ForceGenerator, public ParametrizedObject {
public:
    /**
   * \brief Constructor.
   *
   * \param model The model to use for this spring. A copy of the model will be created and used.
   * \param name  Name of the Spring.
   */
    Spring(const SpringModel & model, const std::string & name);

    /**
   * \brief Destructor.
   */
    virtual ~Spring();

    /**
   * \brief Calculate and apply force of this Spring.
   */
    virtual void applyForce() = 0;

    /**
   * \brief Reset forces introduced by this Spring..
   */
    virtual void resetForce() = 0;

    /**
   * \brief Get current force of this Spring.
   *
   * \return  The spring force.
   */
    TScalar getSpringForce() const;

    TScalar getSpringLength() const; // added by MH

    TScalar getSpringVelocity() const; // added by MH

    /**
   * \brief Get model of this Spring.
   *
   * \return  The spring model.
   */
    SpringModel & getModel();

    /**
   * \brief Get model of this spring.
   *
   * \return  The spring model.
   */
    const SpringModel & getModel() const;

    /**
   * \brief Get name of this Spring.
   *
   * \return  The name.
   */
    virtual const std::string & getName() const;

    /**
   * \brief Get Integrator, if this ForceGenerator is stateful and reguires time-integration.
   *
   * \return Pointer to IIntegrate. Zero if no integration is required.
   */
    virtual IIntegrate * getIntegrator();

protected:
    /// Model of the Spring
    SpringModel * model;

    /// Force of the Spring
    TScalar f;

    /// Length of the Spring          added by MH
    TScalar Lm;

    /// Velocity of the Spring        added by MH
    TScalar Vm;

    USE_PARAMETERS_OF(model)
}; //class Spring

}; //namespace mbslib

#endif
