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
 * \file mbslib/elements/compound/MbsCompoundWithAdder.hpp
 * Declaration of mbslib::MbsCompoundWithAdder
 */
#ifndef __MBSLIB_MBSCOMPOUNDWITHADDER_HPP__
#define __MBSLIB_MBSCOMPOUNDWITHADDER_HPP__

#include <mbslib/elements/MbsCompound.hpp>
namespace mbslib {

/**
 * \brief Mbs compound with adder.
 */
class MbsCompoundWithAdder : public MbsCompound {
public:
    /**
   * \brief Constructor.
   *
   * \param name  (optional) the name.
   */
    MbsCompoundWithAdder(const std::string & name = "");

    virtual ~MbsCompoundWithAdder();
    /**
   * \brief Adds an element.
   *
   * \param element If non-null, the element.
   */
    void addElement(MbsObject * element);

    /**
   * \brief Adds an element.
   *
   * \param element The element.
   */
    void addElement(MbsObject & element);
}; // class MbsCompoundWithAdder

} // namespace mbslib

#endif
