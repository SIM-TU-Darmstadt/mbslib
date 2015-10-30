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

#ifndef __MBSLIB_USER_STATE_VARIABLE_HPP__
#define __MBSLIB_USER_STATE_VARIABLE_HPP__
#include <mbslib/elements/IIntegrate.hpp>
#include <mbslib/elements/MbsObject.hpp>
#include <vector>

namespace mbslib {
class UserStateVariable : public IIntegrate {
private:
    size_t _dimension;
    TVectorX _state;
    TVectorX _dState;
    TVectorX _storedState;
    TVectorX _storedDState;
    std::vector< bool > _unconstrainedAngle;
    std::vector< std::string > _variableNames;
    TVectorX _maxValue;
    TVectorX _minValue;

public:
    UserStateVariable(size_t dimension);
    virtual ~UserStateVariable();

    /**
     * \brief Integrate one timestep.
     *
     * \param dt  The timeintervall to integrate over
     */
    void integrate(TTime dt);

    /**
     * \brief Store current state.
     */
    void storeState();

    /**
     * \brief Go back to last stored state.
     */
    void restoreState();

    /**
     * \brief Get number of state variables.
     *
     * \return  The number of state variables.
     */
    size_t getNumberOfStateVariables() const;

    /**
     * \brief Get state.
     *
     * \return  The state vector.
     */
    TVectorX getState() const;

    /**
      * \brief indicates whether the variable at i is an unconstrained angle
      *
      */
    bool getIsStateVariableUnconstrainedAngle(unsigned int i) const;
    void setIsStateVariableUnconstrainedAngle(unsigned int i, bool value);

    /**
     * \brief Get first derivative of state wrt. time.
     *
     * \return  First derivative of state wrt. time.
     */
    TVectorX getDStateDt() const;
    // set the derived state
    void setDStateDt(const TVectorX & dState);
    // sets a entry in derived state
    void setDStateDt(int i, TScalar value);
    // sets the first value in derived state
    void setDStateDt(TScalar value);

    /**
     * \brief Set state.
     *
     * \param state The state.
     */
    void setState(const TVectorX & state);
    void setState(int i, TScalar value);
    void setState(TScalar value);

    std::string getStateVariableName(unsigned int i) const;
    void setStateVariableName(unsigned int i, const std::string & value);

    void getLowerStateLimits(TVectorX & limits) const;
    void getUpperStateLimits(TVectorX & limits) const;

    void getStateLimits(TVectorX & lower, TVectorX & upper) const;
};
}

#endif
