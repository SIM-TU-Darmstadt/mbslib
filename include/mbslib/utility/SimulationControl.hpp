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
 * \file mbslib/utility/SimulationControl.hpp
 * Declaration of mbslib::SimulationControl
 */
#ifndef __MBSLIB_SIMULATIONCONTROL_HPP__
#define __MBSLIB_SIMULATIONCONTROL_HPP__

#include <mbslib/collision/CollisionDetector.hpp>
#include <mbslib/elements/MbsCompound.hpp>
#include <mbslib/elements/force/ForceGeneratorSet.hpp>

namespace mbslib {

/**
 * \brief Simulation control. \todo kommentieren
 */
class SimulationControl {
public:
    /**
   * \brief Constructor.
   *
   * \param mbs The mbs compound.
   * \param cd  The collision detector.
   */
    SimulationControl(MbsCompound & mbs, CollisionDetector & cd);

    /**
   * \brief Constructor.
   *
   * \param [in,out]  mbs The mbs compound.
   * \param [in,out]  cd  The collisiondetector.
   * \param [in,out]  ef  The force generator set.
   */
    SimulationControl(MbsCompound & mbs, CollisionDetector & cd, ForceGeneratorSet & ef);

    virtual ~SimulationControl();
    /**
   * \brief Do a timestep.
   *
   * \param dt              The intervall.
   * \param maxPenetration  The maximum penetration.
   */
    void doTimestep(TTime dt, TScalar maxPenetration);

    /**
   * \brief Get time of simulation.
   *
   * \return  The time.
   */
    TTime getTime() const {
        return time;
    }

protected:
    /// The mbs
    MbsCompound & mbs;

    /// The collision detector.
    CollisionDetector & cd;

    /// External force set.
    ForceGeneratorSet * extF;

    /// The simulation time
    TTime time;
}; //class SimulationControl

} //namespace mbslib

#endif
