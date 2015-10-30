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
 * \file mbslib/elements/spring/SpringModel.hpp
 * Declaration of mbslib ::SpringModel
 */
#ifndef __MBSLIB_SPRINGMODEL_HPP__
#define __MBSLIB_SPRINGMODEL_HPP__

#include <mbslib/parameter/ParametrizedObject.hpp>
#include <mbslib/utility/types.hpp>

#include <vector>

namespace mbslib {

/**
 * \brief Spring model. \todo kommentieren
 */
class SpringModel : public ParametrizedObject {
public:
    /**
   * \brief Default constructor.
   */
    SpringModel();

    virtual ~SpringModel();

    /**
   * \brief Calculate force along spring.
   *
   * \param l   \todo kommentieren
   * \param dl  \todo kommentieren
   *
   * \return  The calculated force.
   */
    virtual TScalar calculateForce(TScalar l, TScalar dl) const = 0;

    /**
   * \brief Calculate first derivative of force wrt length.
   *
   * \param l   \todo kommentieren
   * \param dl  \todo kommentieren
   *
   * \return  \todo kommentieren
   */
    virtual TScalar calculateDForceDLength(TScalar l, TScalar dl) const = 0;

    /**
   * \brief Calculate first derivative of force wrt length.
   *
   * \param l   \todo kommentieren
   * \param dl  \todo kommentieren
   *
   * \return  \todo kommentieren
   */
    virtual TScalar calculateDForceDVelocity(TScalar l, TScalar dl) const = 0;

    /**
   * \brief Clone model.
   *
   * \return a copy of this object.
   */
    virtual SpringModel * clone() const = 0;

}; //class SpringModel

}; //namespace mbslib

#endif
