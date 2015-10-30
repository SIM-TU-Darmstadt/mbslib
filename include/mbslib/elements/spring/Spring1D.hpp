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
 * \file mbslib/elements/spring/Spring1D.hpp
 * Declaration of mbslib ::Spring1D
 */
#ifndef __MBSLIB_SPRING1D_HPP__
#define __MBSLIB_SPRING1D_HPP__

#include <mbslib/elements/spring/Spring.hpp>

#include <mbslib/elements/joint/Joint1DOF.hpp>
#include <mbslib/elements/spring/SpringModel.hpp>

namespace mbslib {

/**
 * \brief one dimensional Spring. \todo kommentieren
 */
class Spring1D : public Spring {
public:
    /**
   * \brief Constructor.
   *
   * \param model   The model.
   * \param j1      The first Joint1DOF &.
   * \param j2      The second Joint1DOF &.
   * \param name    The name.
   */
    Spring1D(const SpringModel & model, Joint1DOF & j1, Joint1DOF & j2, const std::string & name);

    virtual ~Spring1D();
    /**
   * \brief Apply force of spring to joints.
   */
    void applyForce();

    /**
   * \brief Reset forces of points attached to joints.
   */
    void resetForce();

private:
    // first joint connected to spring
    Joint1DOF & j1;

    // secont joint connected to spring
    Joint1DOF & j2;
}; //class Spring1D

}; //namespace mbslib

#endif
