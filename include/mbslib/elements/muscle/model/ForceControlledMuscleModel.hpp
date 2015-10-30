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
 * \file mbslib/ForceControlledMuscleModel.h
 * Declaration of mbslib ::ForceControlledMuscleModel
 */
#ifndef __mbslib_ForceControlledMuscleModel_HPP__
#define __mbslib_ForceControlledMuscleModel_HPP__

#include <mbslib/elements/muscle/MuscleModel.hpp>

namespace mbslib {

/**
 * \brief Force controlled muscle model.
 */
class ForceControlledMuscleModel : public MuscleModel {
public:
    /**
   * \brief Constructor.
   */
    ForceControlledMuscleModel();

    virtual ~ForceControlledMuscleModel();
    /**
   * \brief Set control value of muscle.
   *  
   *  For this model, this is simply the force created by the muscle.
   *
   * \param c The control value.
   * \param valueId Id of the control value to be set.
   */
    virtual void setControlValue(TScalar c, unsigned int valueId = 0);

    /**
   * \brief Get control value of muscle.
   *  
   *  For this model, this is simply the force created by the muscle.
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
   * \brief Get control value name of muscle. This obviously depends greatly on the type of Muscle Model.
   *
   * \return  The control value name.
   */
    virtual const std::string & getControlValueName(unsigned int valueId = 0) const;

    /**
   * \brief Calculate force along muscle.
   *
   * \param l   Current length of the muscle.
   * \param dl  First derivative of length of muscle wrt time.
   *
   * \return  The calculated force.
   */
    virtual TScalar calculateForce(TScalar l, TScalar dl) const;

    /**
   * \brief Calculate first derivative of force wrt length.
   *
   * \param l   Current length of the muscle.
   * \param dl  First derivative of length of muscle wrt time.
   *
   * \return  The calculated derivative of force.
   */
    virtual TScalar calculateDForceDLength(TScalar l, TScalar dl) const;

    /**
   * \brief Calculate first derivative of force wrt velocity.
   *
   * \param l   Current length of the muscle.
   * \param dl  First derivative of length of muscle wrt time.
   *
   * \return  The calculated derivative of force
   */
    virtual TScalar calculateDForceDVelocity(TScalar l, TScalar dl) const;

    /**
   * \brief Clone model.
   *
   * \return  null if it fails, else a copy of this object.
   */
    virtual SpringModel * clone() const;

private:
    /// \brief force along the muscle.
    TScalar f;

    /**
   * \brief Parameters.
   * \todo Kommentieren
   * \param   The.
   * \param   The.
   * \param   The.
   *
   * \return  .
   */
    START_PARAMETER_LIST
    PARAMETER(force, f, f)
    END_PARAMETER_LIST

}; //class ForceControlledMuscleModel

}; //namespace mbslib

#endif
