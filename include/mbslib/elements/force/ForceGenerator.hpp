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
 * \file mbslib/elements/force/ForceGenerator.hpp
 * Declaration of mbslib ::ForceGenerator
 */
#ifndef __MBSLIB_FORCEGENERATER_HPP__
#define __MBSLIB_FORCEGENERATER_HPP__

#include <mbslib/MbslibBaseClass.hpp>
#include <mbslib/elements/IIntegrate.hpp>

#include <string>

namespace mbslib {

/**
 * \brief ForceGenerator is a model for anything which can introduce a force resp. torque at the mbs.
 *
 * Forces/Torques can be introduced at any joint and any endpoint.
 */
class ForceGenerator {
public:
    virtual ~ForceGenerator();

    /**
   * \brief Apply force of this ForceGenerator to the mbs.
   */
    virtual void applyForce() = 0;

    /**
   * \brief Reset force introduced by this ForceGenerator.
   */
    virtual void resetForce() = 0;

    /**
   * \brief Get Integrator, if this ForceGenerator is stateful and reguires time-integration.
   *
   * \return Pointer to IIntegrate. Zero if no integration is required.
   */
    virtual IIntegrate * getIntegrator() = 0;
}; //class ForceGenerator

} //namespace mbslib

#endif
