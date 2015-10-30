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
 * \file mbslib/elements/object/MbsObjectTwoSucc.hpp
 * Declaration of mbslib ::MbsObjectTwoSucc
 */
#ifndef __MBSLIB_MBSOBJECTTWOSUCC_HPP__
#define __MBSLIB_MBSOBJECTTWOSUCC_HPP__

#include <mbslib/elements/MbsObject.hpp>

namespace mbslib {

/**
 * \brief Mbs object with two successors.
 */
class MbsObjectTwoSucc : public MbsObject {
public:
    /**
   * \brief Check if setup is valid.
   *
   * \return  true if valid, false if not.
   */
    virtual bool isValid() const;

    /**
   * \brief Returns the number of successors this MbsObject has (2)
   *
   * \return  The successor count.
   */
    virtual size_t getSuccessorCount() const;

    /**
   * \brief returns the successor at index i.
   *
   * \param index Zero-based index of the successor.
   *
   * \return  null if it fails, else the successor.
   */
    virtual const MbsObject * getSuccessor(size_t index) const;

    virtual ~MbsObjectTwoSucc();

protected:
    /**
   * \brief Constructor.
   *
   * \param n (optional) the name.
   */
    MbsObjectTwoSucc(const std::string & n = "");

    /**
   * \brief Constructor.
   *
   * \param pred  The predecessor.
   * \param n     (optional) the name.
   */
    MbsObjectTwoSucc(MbsObject & pred, const std::string & n = "");

    /**
   * \brief Set successor of object.
   *
   * \param object   The succeeding object.
   *
   * \return  true if it succeeds, false if it fails.
   */
    virtual bool addSuccessor(MbsObject & object);

    /// The first successor of this object.
    MbsObject * succOne;

    /// The second successor of this object.
    MbsObject * succTwo;

}; // class MbsObjectTwoSucc

} //namespace mbslib

#endif
