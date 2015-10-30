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
 * \file mbslib/elements/spring/model/LinearSpringWithRopeModel.hpp
 * Declaration of mbslib ::LinearSpringWithRopeModel
 */
#ifndef __MBSLIB_LINEARSPRINGWITHROPEMODEL_HPP__
#define __MBSLIB_LINEARSPRINGWITHROPEMODEL_HPP__

#include <mbslib/elements/spring/SpringModel.hpp>

#include <vector>

namespace mbslib {

/**
 * \brief LinearSpringWithRopeModel is a model of linear spring with an added length of massless rope with infite stiffness (if stretched to its length).
 */
class LinearSpringWithRopeModel : public SpringModel {
public:
    /**
   * \brief Constructor.
   *
   * \param k           (optional) Spring coefficient.
   * \param d           (optional) Damoing coefficient.
   * \param ropeLength  (optional) length of the rope.
   */
    LinearSpringWithRopeModel(TScalar k = 0, TScalar d = 0, TScalar ropeLength = 0);

    virtual ~LinearSpringWithRopeModel();
    /**
   * \brief Clone model.
   *
   * \return  a copy of this object.
   */
    virtual SpringModel * clone() const;

    /**
   * \brief Calculates the force.
   *
   * \param l   The length.
   * \param dl  First derivative of length wrt. time.
   *
   * \return  The calculated force.
   */
    virtual TScalar calculateForce(TScalar l, TScalar dl) const;

    /**
   * \brief Calculate first derivative of force wrt length.
   *
   * \param l   The length.
   * \param dl  First derivative of length wrt. time.
   *
   * \return  First derivative of force wrt. length.
   */
    virtual TScalar calculateDForceDLength(TScalar l, TScalar dl) const;

    /**
   * \brief Calculate first derivative of force wrt length.
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

    /// Length of rope part.
    TScalar ropeLength;

    START_PARAMETER_LIST
    PARAMETER(springconstant, k, k)
    PARAMETER(springdamping, d, d)
    PARAMETER(ropeLength, ropeLength, ropeLength)
    END_PARAMETER_LIST
}; //class LinearSpringWithRopeModel

}; //namespace mbslib

#endif
