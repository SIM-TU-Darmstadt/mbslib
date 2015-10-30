/**
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
 * The MBSlib is distributed WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with MBSlib.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \file mbslib/elements/base/Base.hpp
 * Declaration of mbslib::Base
 */
#ifndef __MBSLIB_BASE_HPP__
#define __MBSLIB_BASE_HPP__

#include <mbslib/elements/object/MbsObjectOneSucc.hpp>

namespace mbslib {

/**
 * \brief Base.
 *
 * Representation of the base of the mbs. This is the base class for fixed and
 * free base.
 */
class Base : public MbsObjectOneSucc {
protected:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
   * \brief Constructor.
   *
   * \param name (optional) the name.
   */
    Base(const std::string & name = "");
    virtual ~Base();

public:
    /**
   * \brief Access coordinate frame of base.
   *
   *  Only the base of the mbs allows non-const access to the coordinate frame.
   *
   * \return  The coordinate frame.
   */
    CoordinateFrame & getCoordinateFrame();
}; // class Base

} // namespace mbslib

#endif
