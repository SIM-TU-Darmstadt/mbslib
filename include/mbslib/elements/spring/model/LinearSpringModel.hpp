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
 * \file mbslib/elements/spring/model/LinearSpringModel.hpp
 * Declaration of mbslib ::LinearSpringModel
 */
#ifndef __MBSLIB_LINEARSPRINGMODEL_HPP__
#define __MBSLIB_LINEARSPRINGMODEL_HPP__

#include <mbslib/elements/spring/SpringModel.hpp>

#include <vector>

namespace mbslib {

/**
 * \brief LinearSpringModel is a model of spring with linear force function and linear damping.
 */
class LinearSpringModel : public SpringModel {
public:
    /**
   * \brief Constructor.
   *
   * \param k (optional) Spring constant.
   * \param d (optional) Damping of spring.
   */
    LinearSpringModel(TScalar k = 0, TScalar d = 0);

    virtual ~LinearSpringModel();
    /**
   * \brief Clone model.
   *
   * \return a copy of this object.
   */
    virtual SpringModel * clone() const;

    /**
   * \brief Calculate force along spring.
   *
   * \param l   The length.
   * \param dl  First derivative of length wrt. time.
   *
   * \return  The calculated force.
   */
    virtual TScalar calculateForce(TScalar l, TScalar dl) const;

    /**
   * \brief Calculate first derivative of force wrt. length.
   *
   * \param l   The length.
   * \param dl  First derivative of length wrt. time.
   *
   * \return  First derivative of force wrt. length.
   */
    virtual TScalar calculateDForceDLength(TScalar l, TScalar dl) const;

    /**
   * \brief Calculate first derivative of force wrt velocity.
   *
   * \param l   The length.
   * \param dl  First derivative of length wrt. time.
   *
   * \return  First derivative of force wrt. velocity.
   */
    virtual TScalar calculateDForceDVelocity(TScalar l, TScalar dl) const;

protected:
    /// Spring constant.
    TScalar k;

    /// Spring damping.
    TScalar d;

    START_PARAMETER_LIST
    PARAMETER(springconstant, k, k)
    PARAMETER(springdamping, d, d)
    END_PARAMETER_LIST
}; //class LinearSpringModel

}; //namespace mbslib

#endif
