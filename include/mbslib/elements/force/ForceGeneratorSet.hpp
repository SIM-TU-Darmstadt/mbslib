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
 * \file mbslib/elements/force/ForceGeneratorSet.hpp
 * Declaration of mbslib ::ForceGeneratorSet
 */
#ifndef __MBSLIB_FORCEGENERATORSET_HPP__
#define __MBSLIB_FORCEGENERATORSET_HPP__
#include <mbslib/elements/force/ForceGenerator.hpp>
#include <mbslib/MbslibBaseClass.hpp>

#include <vector>
namespace mbslib {
/**
 * \brief Force generator set.
 *
 * ForceGeneratorSet encapsulates a set of objects of class ForceGenerator and allows to apply all of them.
 */
class ForceGeneratorSet : public virtual MbslibBaseClass {
public:
    /**
   * \brief Add a force generator.
   *
   * \param fg  The ForceGenerator & to add.
   */
    void add(ForceGenerator & fg);

    /**
   * \brief Apply forces.
   */
    void applyForces();

    /**
   * \brief Reset forces.
   */
    void resetForces();
    const std::vector< ForceGenerator * > getGenerators() const;
    virtual ~ForceGeneratorSet();

protected:
    /// Force generators in the set.
    std::vector< ForceGenerator * > generators;
}; //class ForceGeneratorSet
} //namespace mbslib

#endif
