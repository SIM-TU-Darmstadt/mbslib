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
* \file mbslib/elements/IIntegrate.hpp
* Declaration of mbslib::IIntegrate
*/
#ifndef __MBSLIB_IINTEGRATE_HPP__
#define __MBSLIB_IINTEGRATE_HPP__

#include <mbslib/utility/types.hpp>

namespace mbslib {
/**
  * \brief IIntegrate is an interface to any object which holds elements to the mbs' time dependents state.
  *
  * The obvious use of IIntegrate is to perform time-integration of the state.
  * Additionally it allows to access the state and the state's first derivative wrt. time.
  */
class IIntegrate {
public:
    virtual ~IIntegrate();
    /**
    * \brief Integrate one timestep.
    *
    * \param dt  The timeintervall to integrate over
    */
    virtual void integrate(TTime dt) = 0;

    /**
    * \brief Store current state.
    */
    virtual void storeState() = 0;

    /**
    * \brief Go back to last stored state.
    */
    virtual void restoreState() = 0;

    /**
    * \brief Get number of state variables.
    *
    * \return  The number of state variables.
    */
    virtual size_t getNumberOfStateVariables() const = 0;

    /**
    * \brief Get state.
    *
    * \return  The state vector.
    */
    virtual TVectorX getState() const = 0;

    /**
    * \brief indicates whether the variable at i is an unconstrained angle
    *
    */
    virtual bool getIsStateVariableUnconstrainedAngle(unsigned int i) const;

    /**
    * \brief Get first derivative of state wrt. time.
    *
    * \return  First derivative of state wrt. time.
    */
    virtual TVectorX getDStateDt() const = 0;

    /**
    * \brief Set state.
    *
    * \param state The state.
    */
    virtual void setState(const TVectorX & state) = 0;

    virtual std::string getStateVariableName(unsigned int) const;
    virtual void getLowerStateLimits(TVectorX & limits) const;
    virtual void getUpperStateLimits(TVectorX & limits) const;
    TVectorX getLowerStateLimits() const;
    TVectorX getUpperStateLimits() const;

    void getStateLimits(TVectorX & lower, TVectorX & upper) const;

}; //class IIntegrate

} // namespace mbslib

#endif
