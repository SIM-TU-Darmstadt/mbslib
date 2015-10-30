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
 * \file mbslib/elements/Controllable.hpp
 * Declaration of mbslib::Controllable
 */
#ifndef __MBSLIB_ICONTROLLABLE_HPP__
#define __MBSLIB_ICONTROLLABLE_HPP__

#include <mbslib/elements/force/ForceGenerator.hpp>
#include <mbslib/utility/types.hpp>

#include <float.h>
namespace mbslib {

/**
 * \brief Controllable is a special ForceGenerator which can be controlled by external control values.
 *
 * It is also possible to calculate the first derivative of the created force wrt to any control value 
 * and apply this instead of the force.
 */
class Controllable : public virtual ForceGenerator {
public:
    virtual ~Controllable();
    /**
   * \brief Switch to derive-mode.
   *  
   *  In this mode, not the force but the dForce / dControlValue will be calculated. This mode is
   *  used to calculate the sensitifity of a model to the control values.
   *
   * \param dm      Switch for derive mode (true for derive mode on).
   * \param valueId (optional) identifier for the control value.
   */
    virtual void setDeriveMode(bool dm, unsigned int valueId = 0) = 0;

    /**
   * \brief Get derive mode.
   *
   * \return  Current derive mode.
   */
    virtual bool getDeriveMode() const = 0;

    /**
   * \brief Set the control value.
   *
   * \param c       The control value.
   * \param valueId (optional) identifier for the value.
   */
    virtual void setControlValue(TScalar c, unsigned int valueId = 0) = 0;

    /**
   * \brief Get the control value.
   *
   * \param valueId (optional) identifier for the value.
   *
   * \return  The control value.
   */
    virtual TScalar getControlValue(unsigned int valueId = 0) const = 0;

    /**
   * \brief Get number of control values.
   *
   * \return  The number of control values.
   */
    virtual unsigned int getNumberOfControlValues(void) const = 0;

    /**
   * \brief Get name of control value.
   *
   * \param valueId (optional) identifier for the value.
   *
   * \return  The control value name.
   */
    virtual const std::string & getControlValueName(unsigned int valueId = 0) const = 0;

    virtual void getControlLimits(TScalar & lower, TScalar & upper, unsigned int valueId = 0) const;

}; // class Controllable

} // namespace mbslib

#endif
