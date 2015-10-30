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
 * \file mbslib/elements/spring/Spring3D.hpp
 * Declaration of mbslib ::Spring3D
 */
#ifndef __MBSLIB_SPRING3D_HPP__
#define __MBSLIB_SPRING3D_HPP__

#include <mbslib/elements/spring/Spring.hpp>

#include <mbslib/elements/endpoint/Endpoint.hpp>
#include <mbslib/elements/spring/SpringModel.hpp>

#include <vector>

namespace mbslib {

class SpringSegment;

/**
 * \brief Three dimensional Spring.
 */
class Spring3D : public Spring {
public:
    virtual ~Spring3D();
    /**
   * \brief Constructor.
   *
   * \param model   The spring model.
   * \param end1    The end 1.
   * \param end2    The end 2.
   * \param name    The name.
   */
    Spring3D(const SpringModel & model, Endpoint & end1, Endpoint & end2, const std::string & name);

    /**
   * \brief Constructor.
   *
   * \param model   The spring model.
   * \param points  the endpoints.
   * \param name    The name.
   */
    Spring3D(const SpringModel & model, std::vector< Endpoint * > points, const std::string & name);

    /**
   * \brief Constructor.
   *
   * \param model The spring model.
   * \param name  The name.
   */
    Spring3D(const SpringModel & model, const std::string & name);

    /**
   * \brief Add a contact point to the spring.
   *
   * \param ep the endpoint to add
   *
   * \return  the spring with added endpoint.
   */
    Spring3D & operator<<(Endpoint * ep);

    /**
   * \brief Apply force of spring to endpoints.
   */
    void applyForce();

    /**
   * \brief Reset forces of points attached to spring.
   */
    void resetForce();

    // added by MH, for old Stelzer

    /**
   * \brief Switch to derive-mode.
   *
   *  In this mode, not the force but the dForce / dControlValue will be calculated. This mode is
   *  used to calculate the sensitifity of a model to the control values.
   *
   * \param dm      \todo kommentieren
   * \param valueId (optional) identifier for the value.
*/
    //virtual void setDeriveMode(bool dm, unsigned int valueId = 0);

    /**
   * \brief Get derive mode.
   *
   * \return  \todo kommentieren
*/
    //virtual bool getDeriveMode()const;

    std::vector< Endpoint * > & getPoints();

private:
    /// points where the spring is in contact. First and last are fixed, all others are sliding.
    std::vector< Endpoint * > points;

    /// Segments of the spring.
    std::vector< SpringSegment * > segments;

    // added by MH, for old Stelzer
    /*
protected:
  /// derive mode
  bool deriveMode;

  /// derive by control value with id
  unsigned int deriveBy;
*/
    //

}; //class Spring3D

}; //namespace mbslib

#endif
