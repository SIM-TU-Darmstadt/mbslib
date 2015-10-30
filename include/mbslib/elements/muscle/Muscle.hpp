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
 * \file mbslib/elements/muscle/Muscle.hpp
 * Declaration of mbslib ::Muscle
 */
#ifndef __MBSLIB_MUSCLE_HPP__
#define __MBSLIB_MUSCLE_HPP__

#include <mbslib/elements/spring/Spring3D.hpp>
#include <mbslib/elements/Controllable.hpp>

#include <mbslib/elements/muscle/MuscleModel.hpp>

namespace mbslib {

/**
 * \brief Muscle.
 */
class Muscle : public Spring3D, public Controllable {
public:
    /**
   * \brief Constructor.
   *
   * \param model   The muscle model.
   * \param end1    The endpoint 1.
   * \param end2    The endpoint 2.
   * \param name    The name.
   */
    Muscle(const MuscleModel & model, Endpoint & end1, Endpoint & end2, const std::string & name);

    /**
   * \brief Constructor.
   *
   * \param model   The muscle model.
   * \param points  If non-null, the points. \todo kommentieren
   * \param name    The name.
   */
    Muscle(const MuscleModel & model, std::vector< Endpoint * > points, const std::string & name);

    /**
   * \brief Constructor.
   *
   * \param model The muscle model.
   * \param name  The name.
   */
    Muscle(const MuscleModel & model, const std::string & name);

    virtual ~Muscle();
    /**
   * \brief Switch to derive-mode.
   *  
   *  In this mode, not the force but the dForce / dControlValue will be calculated. This mode is
   *  used to calculate the sensitifity of a model to the control values.
   *
   * \param dm      \todo kommentieren
   * \param valueId (optional) identifier for the value.
   */
    virtual void setDeriveMode(bool dm, unsigned int valueId = 0);

    /**
   * \brief Get derive mode.
   *
   * \return  \todo kommentieren
   */
    virtual bool getDeriveMode() const;

    /**
   * \brief Set the control value of the muscle.
   *
   * \param c       The control value.
   * \param valueId (optional) identifier for the value.
   */
    virtual void setControlValue(TScalar c, unsigned int valueId = 0);

    /**
   * \brief Get the control value of the muscle.
   *
   * \param valueId (optional) identifier for the value.
   *
   * \return  The control value.
   */
    virtual TScalar getControlValue(unsigned int valueId = 0) const;

    /**
   * \brief Get number of control values.
   *
   * \return  The number of control values.
   */
    virtual unsigned int getNumberOfControlValues(void) const;

    /**
   * \brief Get name of control value.
   *
   * \param valueId (optional) identifier for the value.
   *
   * \return  The control value name.
   */
    virtual const std::string & getControlValueName(unsigned int valueId = 0) const;
}; //class Muscle

}; //namespace mbslib

#endif
