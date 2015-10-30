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
 * \file mbslib/elements/drive/Spring1DJointDrive.hpp
 * Declaration of mbslib ::Spring1DJointDrive
 */
#ifndef __MBSLIB_SPRING1DJOINTDRIVE_HPP__
#define __MBSLIB_SPRING1DJOINTDRIVE_HPP__

#include <mbslib/elements/spring/Spring.hpp>
#include <mbslib/elements/spring/SpringModel.hpp>

#include <mbslib/elements/Controllable.hpp>

#include <mbslib/elements/joint/Joint1DOF.hpp>

namespace mbslib {

/**
 * \brief A drive which uses a spring model to calculate a force which is applied to the underlying joint
 *        
 */
class Spring1DJointDrive : public Spring, public Controllable {
public:
    /**
   * \brief Constructor.
   *
   * \param model The model.
   * \param j1    The joint.
   * \param name  The name.
   */
    Spring1DJointDrive(const SpringModel & model, Joint1DOF & j1, const std::string & name);

    /**
   * \brief Apply force of spring to joints.
   */
    void applyForce();

    /**
   * \brief Reset forces of points attached to joints.
   */
    void resetForce();

    /**
   * \brief Switch to derive-mode.
   *  
   *  In this mode, not the force but the dForce / dControlValue will be calculated. This mode is
   *  used to calculate the sensitifity of a model to the control values.
   *
   * \param dm      \todo kommentieren
   * \param valueId (optional) identifier for the value.
   */
    virtual void setDeriveMode(bool dm, unsigned int valueId = 0);

    /**
   * \brief Get derive mode.
   *
   * \return  \todo kommentieren
   */
    virtual bool getDeriveMode() const;

    /**
   * \brief Set the control value of the muscle.
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

    virtual ~Spring1DJointDrive();

private:
    /// joint connected to spring
    Joint1DOF & j1;

    /// position of motor
    TScalar position;

    /// velocity of motor;
    TScalar velocity;

    /// derive mode
    bool deriveMode;

    /// derive by control value with id
    unsigned int deriveBy;
}; //class Spring1DJointDrive

}; //namespace mbslib

#endif
