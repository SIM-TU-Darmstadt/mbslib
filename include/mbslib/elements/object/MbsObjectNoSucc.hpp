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
 * \file mbslib/elements/object/MbsObjectNoSucc.hpp
 * Declaration of mbslib ::MbsObjectNoSucc
 */
#ifndef __MBSLIB_MBSOBJECTNOSUCC_HPP__
#define __MBSLIB_MBSOBJECTNOSUCC_HPP__

#include <mbslib/elements/MbsObject.hpp>

namespace mbslib {

/**
 * \brief Mbs object with no successor.
 */
class MbsObjectNoSucc : public MbsObject {
protected:
    /**
   * \brief Constructor.
   *
   * \param n (optional) the n.
   */
    MbsObjectNoSucc(const std::string & n = "");

    /**
   * \brief Constructor.
   *
   * \param pred  The predecessor.
   * \param n     (optional) the n.
   */
    MbsObjectNoSucc(MbsObject & pred, const std::string & n = "");

    virtual ~MbsObjectNoSucc();

    /**
   * \brief Adds a successor.
   *
   * \param object  The object.
   *
   * \return  true if it succeeds, false if it fails.
   */
    virtual bool addSuccessor(MbsObject & object);

    /**
   * \brief Gets the successor count (0).
   *
   * \return  The successor count.
   */
    virtual size_t getSuccessorCount() const;

    /**
   * \brief Returns the Successor at index i (this instace always returns 0)
   *
   * \param i Zero-based index of the sucessor.
   *
   * \return  null if it fails, else the successor.
   */
    virtual const MbsObject * getSuccessor(size_t /*i*/) const;
}; // class MbsObjectNoSucc

} //namespace mbslib

#endif
