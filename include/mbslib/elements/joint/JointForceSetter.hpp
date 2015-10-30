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
 * \file mbslib/elements/joint/JointForceSetter.hpp
 * Declaration of mbslib ::JointForceSetter
 */
#ifndef __MBSLIB_JOINTFORCESETTER_HPP__
#define __MBSLIB_JOINTFORCESETTER_HPP__

#include <mbslib/elements/Controllable.hpp>
#include <mbslib/elements/joint/Joint1DOF.hpp>

namespace mbslib {

/**
 * \brief Joint force setter.
 */
class JointForceSetter : public Controllable {
public:
    /**
   * \brief Constructor.
   *
   * \param j1  The 1D joint
   */
    JointForceSetter(Joint1DOF & j1);

    /**
   * \brief Constructor.
   *
   * \param j1    The 1D joint
   * \param name  The name.
   */
    JointForceSetter(Joint1DOF & j1, const std::string & name);

    virtual ~JointForceSetter();
    /**
   * \brief Apply force to joint.
   */
    void applyForce();

    /**
   * \brief Reset force at joint.
   */
    void resetForce();

    /**
   * \brief Switch to derive-mode.
   *  
   *  In this mode, not the force but the dForce / dControlValue will be calculated. This mode is
   *  used to calculate the sensitifity of a model to the control values.
   *
   * \param dm      true to dm. \todo kommentieren
   * \param valueId (optional) identifier for the value. \todo kommentieren
   */
    virtual void setDeriveMode(bool dm, unsigned int valueId = 0);

    /**
   * \brief Get derive mode.
   *
   * \return  true if derive mode else false.
   */
    virtual bool getDeriveMode() const;

    /**
   * \brief Set the control value.
   *  
   *  In this case this is simply the force to set for the joint.
   *
   * \param c       The control value.
   * \param valueId (optional) identifier for the value.
   */
    virtual void setControlValue(TScalar c, unsigned int valueId = 0);

    /**
   * \brief Get the control value of the muscle.
   *
   * \param valueId (optional) identifier for the value.
   *
   * \return  The control value.
   */
    virtual TScalar getControlValue(unsigned int valueId = 0) const;

    /**
   * \brief Get number of control values.
   *
   * \return  The number of control values.
   */
    virtual unsigned int getNumberOfControlValues(void) const;

    /**
   * \brief Get name of control value.
   *
   * \param valueId (optional) identifier for the value.
   *
   * \return  The control value name.
   */
    virtual const std::string & getControlValueName(unsigned int valueId = 0) const;

    /**
   * \brief Get name of this.
   *
   * \return  The name.
   */
    virtual const std::string & getName() const;

    /**
   * Get integrator. As this class is stateless, we return NULL.
   *
   * \return Always NULL.
   */
    IIntegrate * getIntegrator();

private:
    /// joint connected to this.
    Joint1DOF & j1;

    /// derive mode
    bool deriveMode;

    /// force to use in joint.
    TScalar tau;

    /// name of this.
    std::string name;

}; //class JointForceSetter

}; //namespace mbslib

#endif
