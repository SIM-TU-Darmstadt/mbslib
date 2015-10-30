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
 * \file mbslib/elements/spring/Spring1DSingleEnded.hpp
 * Declaration of mbslib ::Spring1DSingleEnded
 */
#ifndef __MBSLIB_SPRING1DSINGLEENDED_HPP__
#define __MBSLIB_SPRING1DSINGLEENDED_HPP__

#include <mbslib/elements/spring/Spring.hpp>

#include <mbslib/elements/joint/Joint1DOF.hpp>
#include <mbslib/elements/spring/SpringModel.hpp>

namespace mbslib {

/**
 * \brief A spring attached to a joint.
 *  calculates the retention force by using the difference of the joint position to the joints initial position (0)
 *  
 */
class Spring1DSingleEnded : public Spring {
public:
    virtual ~Spring1DSingleEnded();
    /**
   * \brief Constructor.
   *
   * \param model   The spring model.
   * \param j1      The joint
   * \param name    The name.
   */
    Spring1DSingleEnded(const SpringModel & model, Joint1DOF & j1, const std::string & name);

    /**
   * \brief Apply force of spring to joints.
   */
    void applyForce();

    /**
   * \brief Reset forces of points attached to joints.
   */
    void resetForce();

private:
    /// joint connected to spring
    Joint1DOF & j1;
}; //class Spring1DSingleEnded

}; //namespace mbslib

#endif
