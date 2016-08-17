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

#include <mbslib/elements/muscle/Muscle.hpp>
#include <mbslib/elements/muscle/MuscleModel.hpp>

#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <math.h>

#ifndef __MUSCLE_WITH_STATE_MODEL_HPP__
#define __MUSCLE_WITH_STATE_MODEL_HPP__

//using namespace std;
using namespace mbslib;

class MuscleWithStateModel : public MuscleModel, public IIntegrate {
public:
    /**
     * \brief Constructor.
     */
    MuscleWithStateModel();
    virtual ~MuscleWithStateModel();
    /**
   * \brief Switch to derive-mode.
   *
   *  In this mode, not the force but the dForce / dControlValue will be calculated. This mode is
   *  used to calculate the sensitifity of a model to the control values.
   *
   * \param dm  Switch for derive mode (true == on).
   * \param valueId Number of control value to derive by.
   */
    virtual void setDeriveMode(bool dm, unsigned int valueId = 0);

    /**
   * \brief Get derive mode.
   *
   * \return  Derive mode (true == on).
   */
    virtual bool getDeriveMode() const;

    /**
   * \brief Set control value of muscle.
   *
   *  For this model, this is simply the force created by the muscle.
   *  Inherited via MuscleModel.
   *
   * \param c The control value.
   * \param valueId Id of the control value to be set.
   */
    virtual void setControlValue(TScalar c, unsigned int valueId = 0);

    /**
   * \brief Get control value of muscle.
   *
   *  For this model, this is simply the force created by the muscle.
   *  Inherited via MuscleModel.
   *
   * \return  The control value.
   */
    virtual TScalar getControlValue(unsigned int valueId = 0) const;

    /**
   * \brief Get number of control values.
   *
   *  Inherited via MuscleModel.
   *
   * \return  The number of control values.
   */
    virtual unsigned int getNumberOfControlValues(void) const;

    /**
   * \brief Get control value name of muscle. This obviously depends greatly on the type of Muscle Model.
   *
   *  Inherited via MuscleModel.
   *
   * \return  The control value name.
   */
    virtual const std::string & getControlValueName(unsigned int valueId = 0) const;

    /**
   * \brief Calculate force along muscle.
   *
   *  Inherited via MuscleModel.
   *
   * \param l   Current length of the muscle.
   * \param dl  First derivative of length of muscle wrt time.
   *
   * \return  The calculated force.
   */
    virtual TScalar calculateForce(TScalar l, TScalar dl) const;

    /**
   * \brief Calculate first derivative of force wrt length.
   *
   *  Inherited via MuscleModel.
   *
   * \param l   Current length of the muscle.
   * \param dl  First derivative of length of muscle wrt time.
   *
   * \return  The calculated derivative of force.
   */
    virtual TScalar calculateDForceDLength(TScalar l, TScalar dl) const;

    /**
   * \brief Calculate first derivative of force wrt velocity.
   *
   *  Inherited via MuscleModel.
   *
   * \param l   Current length of the muscle.
   * \param dl  First derivative of length of muscle wrt time.
   *
   * \return  The calculated derivative of force
   */
    virtual TScalar calculateDForceDVelocity(TScalar l, TScalar dl) const;

    /**
   * \brief Clone model.
   *
   *  Inherited via MuscleModel.
   *
   * \return  null if it fails, else a copy of this object.
   */
    virtual SpringModel * clone() const;

    /**
   * \brief Integrate one timestep.
   *
   *  Inherited via IIntegrate.
   *
   * \param dt  The timeintervall to integrate over
   */
    virtual void integrate(TTime dt);

    /**
   * \brief Store current state.
   *
   *  Inherited via IIntegrate.
   */
    virtual void storeState();

    /**
   * \brief Go back to last stored state.
   *
   *  Inherited via IIntegrate.
   */
    virtual void restoreState();

    /**
   * \brief Get number of state variables.
   *
   *  Inherited via IIntegrate.
   *
   * \return  The number of state variables.
   */
    virtual size_t getNumberOfStateVariables() const;

    /**
   * \brief Get state.
   *
   *  Inherited via IIntegrate.
   *
   * \return  The state vector.
   */
    virtual TVectorX getState() const;

    /**
   * \brief Get first derivative of state wrt. time.
   *
   *  Inherited via IIntegrate.
   *
   * \return  First derivative of state wrt. time.
   */
    virtual TVectorX getDStateDt() const;

    /**
   * \brief Set state.
   *
   * \param state The state.
   */
    virtual void setState(const TVectorX & state);

    // As we have no parameters for this muscle, we must add an empty paramter list.
    // For other cases, take a look at ParametricedObject.h in the mbslib.
    EMPTY_PARAMETER_LIST

protected:
    /// Switch for derive-mode
    bool deriveMode;

    /// Id of value to derive by.
    unsigned int deriveByValueId;

    /// Current force of muscle (this is the state).
    TScalar force;

    /// Current control value (this example has only one).
    TScalar controlValue;

    /// Stored state
    TScalar storedForce;

    /// Stored first derivative of state wrt. time.
    TScalar stored_dForce_dT;

}; // class MuscleWithStateModel

#endif // __MUSCLE_WITH_STATE_MODEL_HPP__
