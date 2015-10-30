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
 * \file mbslib/elements/joint/PrismaticJoint.hpp
 * Declaration of mbslib::PrismaticJoint
 */
#ifndef __MBSLIB_PRISMATICJOINT_HPP__
#define __MBSLIB_PRISMATICJOINT_HPP__

#include <mbslib/elements/joint/Joint1DOF.hpp>

namespace mbslib {

/**
 * \brief Prismatic joint.
 */
class PrismaticJoint : public Joint1DOF {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
   * \brief Constructor.
   *
   * \param pred                The predecessor.
   * \param direction           The direction.
   * \param jointId             Identifier for the joint.
   * \param stateVectorPosition The state vector position.
   * \param jointOffset         (optional) the joint offset.
   * \param jointFriction       (optional) the joint friction.
   * \param gearRatio           (optional) the gear ratio.
   * \param rotorInertia        (optional) the rotor inertia.
   * \param name                (optional) the name.
   */
    PrismaticJoint(MbsObject & pred, const TVector3 & direction, int jointId, int stateVectorPosition, TScalar jointOffset = 0, TScalar jointFriction = 0, TScalar gearRatio = 0, TScalar rotorInertia = 0, const std::string & name = "");
    virtual ~PrismaticJoint();
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

    /**
   * \brief Calculate CRBA force if this joint is accelerated to move sub-tree.
   *
   * \param f The force.
   * \param n The n.\todo kommentieren
   *
   * \return  The calculated crb aforce.
   */
    virtual TScalar calculateCRBAforce(TVector3 & f, TVector3 & n);

    /**
   * \brief Calculate CRBA force if a joint down the chain is accelerated leading to force f / n at
   *  pos.
   *
   * \param pos The position.
   * \param f   The force.
   * \param n   The n.\todo kommentieren
   *
   * \return  The calculated crba force.
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
   * \brief Returns the direction to which this prismatic joint is initialized.
   *
   * \return  The direction.
   */
    const TVector3 & getDirection() const;

protected:
    /**
   * \brief Calculate relative pose of this element.
   */
    virtual void calcRelPose(TVector3 & relr, TMatrix3x3 & relR);

    /// The direction of the joint
    TVector3 dir;

    /// temporary value calculated by dirkin
    //TVector3 relr;

}; // class PrismaticJoint

} // namespace mbslib

#endif
