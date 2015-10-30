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
 * \file mbslib/elements/joint/Joint1DOF.hpp
 * Declaration of mbslib::Joint1DOF
 */
#ifndef __MBSLIB_JOINT1DOF_HPP__
#define __MBSLIB_JOINT1DOF_HPP__

#include <mbslib/elements/joint/JointNDOF.hpp>

namespace mbslib {

/**
 * \brief Joint with 1 degree of freedom. \todo kommentieren
 */
class Joint1DOF : public JointNDOF< 1 > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
   * \brief Get joint state.
   *
   * \return The joint state jp
   */
    const JointState & getJointState() const;

    /**
   * \brief Get joint state.
   *
   * \return  The joint state jp.
   */
    JointState & getJointState();

    /**
   * \brief Get the joint position.
   *
   * \return The joint position q.
   */
    TScalar getJointPosition();

    /**
   * \brief Get the joint velocity.
   *
   * return The joint velocity dotq.
   */
    TScalar getJointVelocity();

    /**
   * \brief Get the joint acceleration.
   *
   * \return The joint acceleration ddotq.
   */
    TScalar getJointAcceleration();

    /**
   * \brief Get the joints lower limit.
   *
   */
    TScalar getJointLowerPositionLimit() const;
    /**
   * \brief Gets the joints upper limit.
   *
   */
    TScalar getJointUpperPositionLimit() const;
    /**
   * \brief Sets the joints lower limit.
   *
   */
    void setJointLowerPositionLimit(TScalar value);

    /**
   * \brief Sets the joints upper limit.
   *
   */
    void setJointUpperPositionLimit(TScalar value);

    void setJointVelocityLimit(TScalar value);
    void setJointEffortLimit(TScalar value);
    TScalar getJointVelocityLimit() const;
    TScalar getJointEffortLimit() const;

    virtual void getLowerStateLimits(TVectorX & limits) const;
    virtual void getUpperStateLimits(TVectorX & limits) const;

    virtual std::string getStateVariableName(unsigned int i) const;

    void enforceLimits();

    /**
   * \brief Set the joint position.
   *
   * \param q The joint position q.
   */
    void setJointPosition(TScalar q);

    /**
   * \brief Set the joint velocity.
   *
   * \param dotq The joint velocity dotq.
   */
    void setJointVelocity(TScalar dotq);

    /**
   * \brief Set the joint acceleration.
   *
   * \param ddotq The joint acceleration ddotq.
   */
    void setJointAcceleration(TScalar ddotq);

    /**
   * \brief Get force of joint.
   *
   * \return  The joint force|torque.
   */
    TScalar getJointForceTorque() const;

    /**
   * \brief Set force of joint.
   *
   * \param tau_joint The force at joint.
   */
    void setJointForceTorque(TScalar tau_joint);

    /**
   * \brief Get force of joint.
   *
   * \return The external joint force torque.
   */
    TScalar getExternalJointForceTorque() const;

    /**
   * \brief Set force of joint.
   *
   * \param tau_joint The joint force.
   */
    void setExternalJointForceTorque(TScalar tau_joint);

    /**
   * \brief Calculate CRBA force if this joint is accelerated to move sub-tree.
   *
   * \param f The force.
   * \param n The n.\todo kommentieren
   *
   * \return  The calculated crb aforce.
   */
    virtual TScalar calculateCRBAforce(TVector3 & f, TVector3 & n) = 0;

    /**
   * \brief Calculate CRBA force if a joint down the chain is accelerated leading to force f / n at
   *  pos.
   *
   * \param pos The position.
   * \param f   The force.
   * \param n   The .\todo kommentieren
   *
   * \return  The calculated crb aforce.
   */
    virtual TScalar calculateCRBAforce(const TVector3 & pos, const TVector3 & f, const TVector3 & n) = 0;

    /**
   * \brief Integrate one timestep.
   *
   * \param dt  the intervall to integrate over
   */
    virtual void integrate(TTime dt);

    /**
   * \brief Store current state.
   */
    virtual void storeState();

    /**
   * \brief Go back to last stored state.
   */
    virtual void restoreState();

    /**
   * \brief Get number of state variables.
   *
   * \return  The number of state variables.
   */
    virtual size_t getNumberOfStateVariables() const;

    /**
   * \brief Get state.
   *
   * \return  The state.
   */
    virtual TVectorX getState() const;

    /**
   * \brief Get first derivative of state w.r.t. to time.
   *
   * \return  The derivative of state w.r.t. dt.
   */
    virtual TVectorX getDStateDt() const;

    /**
   * \brief Set state.
   *
   * \param   The state vector.
   */
    virtual void setState(const TVectorX &);

    virtual ~Joint1DOF();

protected:
    /**
   * \brief Constructor.
   *
   * \param pred                 The predicate.
   * \param jointId             Identifier for the joint.
   * \param stateVectorPosition The state vector position.
   * \param jointOffset         (optional) the joint offset.
   * \param jointFriction       (optional) the joint friction.
   * \param gearRatio           (optional) the gear ratio.
   * \param rotorInertia        (optional) the rotor inertia.
   * \param name                (optional) the name.
   */
    Joint1DOF(MbsObject & pred, int jointId, int stateVectorPosition, TScalar jointOffset = 0, TScalar jointFriction = 0, TScalar gearRatio = 0, TScalar rotorInertia = 0, const std::string & name = "");

    /**
   * \brief Gets tau.
   *
   * \return  \todo kommentieren
   */
    virtual TVectorN abaGetTau() {
        TVectorN t;
        t(0) = tau;
        return t;
    }

    /**
   * \brief Getsdot q.
   *
   * \return  \todo kommentieren .
   */
    virtual TVectorN abaGetDotQ() {
        TVectorN dq;
        dq(0) = jp.dotq;
        return dq;
    }

    /**
   * \brief Gets d dot q.
   *
   * \return  \todo kommentieren .
   */
    virtual TVectorN abaGetDDotQ() {
        TVectorN ddq;
        ddq(0) = jp.ddotq;
        return ddq;
    }

    /**
   * \brief Aba set tau.
   *
   * \param t \todo kommentieren.
   */
    virtual void abaSetTau(const TVectorN & t);

    /**
   * \brief Aba set dot q.
   *
   * \param dotq  \todo kommentieren.
   */
    virtual void abaSetDotQ(const TVectorN & dotq);

    /**
   * \brief Aba set d dot q.
   *
   * \param ddotq \todo kommentieren
   */
    virtual void abaSetDDotQ(const TVectorN & ddotq);
    /*
  /// id of the joint
  int jointId;
  
  /// the preceeding joint
  Joint1DOF * predJoint;
  
  /// position of joint's variables in statevector
  int stateVectorPosition;
  */

    TScalar lowerPositionLimit;
    TScalar upperPositionLimit;
    TScalar effortLimit;
    TScalar velocityLimit;
    /// Offset of joint.
    TScalar jointOffset;

    /// Position of joints and its derivatives
    JointState jp;

    /// Stored position
    JointState storedJP;

    /// Force/Torqu acting in joint as calculated by RNE.
    TScalar tau;

    /// External Torque acting at joint.
    TScalar externalTau;

    /// Friction of the joint
    TScalar jointFriction;

    /// Gears
    TScalar gearRatio;

    /// Rotor
    TScalar rotorInertia;

    /// Rotor with squared gears
    TScalar squaredGearRotorInertia;

    /*
  /// joint axis representation for ABA
  TVector6 S;
  
  /// temporary value for ABA, calcualted in sweep one, used in sweeps two and three
  TVector6 c;
  
  /// temporary value for ABA, calcualted in sweep one, used in sweeps two and three
  TVector6 U;
  
  /// temporary value for ABA, calcualted in sweep one, used in sweeps two and three
  TScalar D;
  
  /// temporary value for ABA, calcualted in sweep one, used in sweeps two and three
  TScalar u;
  */

    START_PARAMETER_LIST
    PARAMETER(jointFriction, jointFriction, jointFriction);
    PARAMETER(rotorInertia, rotorInertia, rotorInertia);
    PARAMETER(q, jp.q, jp.q)
    PARAMETER(dq, jp.dotq, jp.dotq)
    PARAMETER(ddq, jp.ddotq, jp.ddotq)
    PARAMETER(tau, tau, tau)
    PARAMETER(lowerPositionLimit, lowerPositionLimit, lowerPositionLimit);
    PARAMETER(upperPositionLimit, upperPositionLimit, upperPositionLimit);
    PARAMETER(velocityLimit, velocityLimit, velocityLimit);
    PARAMETER(effortLimit, effortLimit, effortLimit);
    END_PARAMETER_LIST
}; // class Joint

} // namespace mbslib

#endif
