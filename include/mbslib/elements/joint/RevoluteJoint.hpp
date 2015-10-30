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
 * \file mbslib/elements/joint/RevoluteJoint.hpp
 * Declaration of mbslib::RevoluteJoint
 */
#ifndef __MBSLIB_REVOLUTEJOINT_HPP__
#define __MBSLIB_REVOLUTEJOINT_HPP__

#include <mbslib/elements/joint/Joint1DOF.hpp>

namespace mbslib {

/**
 * \brief Revolute joint. \todo kommentieren
 */
class RevoluteJoint : public Joint1DOF {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
   * \brief Constructor.
   *
   * \param pred                The predecessor.
   * \param axis                The axis.
   * \param jointId             Identifier for the joint.
   * \param stateVectorPosition The state vector position.
   * \param jointOffset         (optional) the joint offset.
   * \param jointFriction       (optional) the joint friction.
   * \param gearRatio           (optional) the gear ratio.
   * \param rotorInertia        (optional) the rotor inertia.
   * \param name                (optional) the name.
   */
    RevoluteJoint(MbsObject & pred, const TVector3 & axis, int jointId, int stateVectorPosition, TScalar jointOffset = 0, TScalar jointFriction = 0, TScalar gearRatio = 0, TScalar rotorInertia = 0, const std::string & name = "");

    virtual ~RevoluteJoint();
    /**
   * \brief Calculated positon part of direct kinematics.
   */
    //virtual void doPosition();

    /**
   * \brief Calculated velocity part of direct kinematics.
   */
    virtual void doVelocity();

    /**
   * \brief Calculate acceleration part of direct kinematics.
   */
    virtual void doAcceleration();

    /**
   * \brief Calculate 2nd sweep of Inverse Dynamics.
   *
   * \param withExternalForce true to take into account the external force.
   */
    virtual void doRneInward(bool withExternalForce);

    /**
   * \brief Calculate aggregate body inwards.
   */
    virtual void doAggregateBody();

    // if the joint is unconstrained this should return true for state i == 0
    //
    virtual bool getIsStateVariableUnconstrainedAngle(unsigned int /*i*/);

    /**
   * \brief Calculate CRBA force if this joint is accelerated to move sub-tree.
   *
   * \param f The f. \todo kommentieren
   * \param n The n. \todo kommentieren
   *
   * \return  The calculated crb aforce.
   */
    virtual TScalar calculateCRBAforce(TVector3 & f, TVector3 & n);

    /**
   * \brief Calculate CRBA force if a joint down the chain is accelerated leading to force f / n at
   *  pos.
   *
   * \param pos The position.
   * \param f   The f. \todo kommentieren
   * \param n   The n. \todo kommentieren
   *
   * \return  The calculated crb aforce.
   */
    virtual TScalar calculateCRBAforce(const TVector3 & pos, const TVector3 & f, const TVector3 & n);

    /**
   * \brief First (outward) sweep of ABA.
   */
    virtual void doABASweep1fwd();

    /**
   * \brief Second (inward) sweep of ABA.
   */
    virtual void doABASweep2fwd();

    /**
   * \brief Third (outward) sweep of ABA.
   */
    virtual void doABASweep3fwd();

    /**
   * \brief First (outward) sweep of inverse dynamics ABA.
   */
    virtual void doABASweep1inv();

    /**
   * \brief Second (inward) sweep of inverse dynamics ABA.
   */
    virtual void doABASweep2inv();

    /**
   * \brief Third (outward) sweep of inverse dynamics ABA.
   */
    virtual void doABASweep3inv();

    /**
   * \brief Make columns of jacobian for a given position.
   *
   * \param [out]  jacobian  The jacobian.
   * \param referencePoint   The reference point.
   */
    virtual void makeJacobian(TMatrix6xX & jacobian, const TVector3 & referencePoint) const;

    /**
   * \brief returns the axis to which this joint was initialized to revolve around.
   *
   * \return  The joint axis.
   */
    const TVector3 & getAxis() const;

protected:
    /**
   * \brief Calculate relative pose of this element.
   *
   * \param relr  The relative position.
   * \param relR  The relative rotation.
   */
    virtual void calcRelPose(TVector3 & relr, TMatrix3x3 & relR);

    /// Axis of rotation of joint.
    TVector3 axis;

    /// Relative orientation as calculated by doPositions;
    //TMatrix3x3 relR;

}; // class RevoluteJoint

} // namespace mbslib

#endif
