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
 * \file mbslib/elements/muscle/MuscleModel.hpp
 * Declaration of mbslib ::MuscleModel
 */
#ifndef __MBSLIB_MUSCLEMODEL_HPP__
#define __MBSLIB_MUSCLEMODEL_HPP__

#include <mbslib/elements/spring/SpringModel.hpp>

namespace mbslib {

/**
 * \brief Muscle model.
 */
class MuscleModel : public SpringModel {
public:
    /**
   * \brief Constructor.
   */
    MuscleModel();
    virtual ~MuscleModel();

    /**
   * \brief Switch to derive-mode.
   *  
   *  In this mode, not the force but the dForce / dControlValue will be calculated. This mode is
   *  used to calculate the sensitifity of a model to the control values.
   *
   * \param dm  Switch for derive mode (true == on).
   * \param valueId Number of control value to derive by.
   */
    void setDeriveMode(bool dm, unsigned int valueId = 0);

    /**
   * \brief Get derive mode.
   *
   * \return  Derive mode (true == on).
   */
    bool getDeriveMode() const;

    /**
   * \brief Set control value of muscle. This obviously depends greatly on the type of Muscle Model.
   *
   * \param c The control value.
   */
    virtual void setControlValue(TScalar c, unsigned int valueId) = 0;

    /**
   * \brief Get control value of muscle. This obviously depends greatly on the type of Muscle Model.
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
   * \brief Get control value name of muscle. This obviously depends greatly on the type of Muscle Model.
   *
   * \return  The control value name.
   */
    virtual const std::string & getControlValueName(unsigned int valueId = 0) const = 0;

private:
    /// true to enable derive mode, false to disable it
    bool deriveMode;

}; //class MuscleModel

}; //namespace mbslib

#endif
