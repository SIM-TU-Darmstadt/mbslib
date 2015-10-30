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
 * \file mbslib/elements/rigidbody/RigidLink.hpp
 * Declaration of mbslib::RigidLink
 */
#ifndef __MBSLIB_RIGIDLINK_HPP__
#define __MBSLIB_RIGIDLINK_HPP__

#include <mbslib/elements/object/MbsObjectOneSucc.hpp>

#include <mbslib/elements/rigidbody/RigidBodyDescription.hpp>

namespace mbslib {

/**
 * \brief Rigid link. \todo kommentieren
 */
class RigidLink : public MbsObjectOneSucc {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
   * \brief Constructor.
   *
   * \param pred            The predecessor.
   * \param relr            The relative position.
   * \param com             The center of mass (relative to relr).
   * \param m               The mass.
   * \param I               The intertia tensor.
   * \param name            (optional) the name.
   */
    RigidLink(MbsObject & pred, const TVector3 & relr, const TVector3 & com, TScalar m, const TMatrix3x3 & I, const std::string & name = "");

    /**
   * \brief Constructor.
   *
   * \param pred                  The predecessor.
   * \param rigidBodyDescription  Information describing the rigid body.
   * \param name                  (optional) the name.
   */
    RigidLink(MbsObject & pred, const RigidBodyDescription & rigidBodyDescription, const std::string & name = "");

    virtual ~RigidLink();
    /**
   * \brief Get DOF of element.
   *
   * \return  The degrees of freedom.
   */
    virtual int getDOF() const;

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
   * Calculate aggregate body inwards.
   */
    virtual void doAggregateBody();

    /**
   * \brief Get modeling information on relative translation.
   *
   * \return  The relative position.
   */
    //const TVector3 & getRelr()const{return fixedRelr;}

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
   * \param [out]     jacobian  The jacobian.
   * \param referencePoint      The reference point.
   */
    virtual void makeJacobian(TMatrix6xX & /*jacobian*/, const TVector3 & /*referencePoint*/) const;

    /**
   * \brief Integrate one timestep.
   *  
   *  This will only effect state-variables (joint position/velocity and pose/velocity of free base)!
   *
   * \param dt  The intervall to integrate over.
   */
    virtual void integrate(TTime /*dt*/) {
    }

    /**
   * \brief Store current state.
   */
    virtual void storeState() {
    }

    /**
   * \brief Go back to last stored state.
   */
    virtual void restoreState() {
    }

    /**
   * \brief returns the mass of this rigid link.
   *
   * \return  The mass.
   */
    const TScalar & getMass() const;

    /**
   * \brief returns the inertia tensor of this rigid link.
   *
   * \return  The inertia tensor.
   */
    const TMatrix3x3 & getInertiaTensor() const;

    /**
   * \brief returns the center of mass vector.
   *
   * \return  The center of mass.
   */
    const TVector3 & getCenterOfMass() const;

    /**
   * \brief returns the relativ position.
   *
   * \return  The fixed relative position.
   */
    const TVector3 & getFixedRelativePosition() const;

protected:
    /**
   * \brief Calculate relative pose of this element.
   *
   * \param relr  The relative position.
   * \param relR  The relative rotation.
   */
    virtual void calcRelPose(TVector3 & relr, TMatrix3x3 & relR);

    /// relative tranlation from predecessor to own cof in predecessors cof (orientation is equal for both cofs).
    TVector3 fixedRelr;

    /// position of center of mass relative to _own_ cof.
    TVector3 com;

    /// mass
    TScalar m;

    /// inertia tensor
    TMatrix3x3 I;

    /// inertia for ABA
    TMatrix6x6 abaI;

    /// temporary value for ABA, calcualted in sweep one, used in sweeps two and three
    TVector6 p;

    /// temporary value for ABA, calcualted in sweep one, used in sweeps two and three
    TVector6 U;

    /// temporary value for ABA, calcualted in sweep one, used in sweeps two and three
    TScalar D;

    /// temporary value for ABA, calcualted in sweep one, used in sweeps two and three
    TScalar u;

    START_PARAMETER_LIST
    PARAMETER(rx, fixedRelr(0), fixedRelr(0))
    PARAMETER(ry, fixedRelr(1), fixedRelr(1))
    PARAMETER(rz, fixedRelr(2), fixedRelr(2))
    PARAMETER(comx, com(0), com(0))
    PARAMETER(comy, com(1), com(1))
    PARAMETER(comz, com(2), com(2))
    PARAMETER(mass, m, m);
    PARAMETER(Ixx, I(0, 0), I(0, 0));
    PARAMETER(Iyy, I(1, 1), I(1, 1));
    PARAMETER(Izz, I(2, 2), I(2, 2));
    PARAMETER(Ixy, I(0, 1), I(0, 1) = I(1, 0));
    PARAMETER(Ixz, I(0, 2), I(0, 2) = I(2, 0));
    PARAMETER(Iyz, I(1, 2), I(1, 2) = I(2, 1));
    END_PARAMETER_LIST
}; // class RigidLink

} // namespace mbslib

#endif
