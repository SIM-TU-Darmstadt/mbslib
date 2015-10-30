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
 * \file mbslib/elements/fixed/FixedRotation.hpp
 * Declaration of mbslib::FixedRotation
 */
#ifndef __MBSLIB_FIXEDROTATION_HPP__
#define __MBSLIB_FIXEDROTATION_HPP__

#include <mbslib/elements/object/MbsObjectOneSucc.hpp>

namespace mbslib {

/**
 * \brief Fixed rotation.
 *
 * Representation of a rotation within the mbs which can not be changed due to motion of the mbs.
 */
class FixedRotation : public MbsObjectOneSucc {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
   * \brief Constructor.
   *
   * \param pred  The predecessor.
   * \param relR  The relative r.
   * \param name  (optional) the name.
   */
    FixedRotation(MbsObject & pred, const TMatrix3x3 & relR, const std::string & name = "");

    /**
   * \brief Get DOF of element.
   *
   * \return  The degrees of freedom.
   */
    virtual int getDOF() const;

    /**
   * \brief Returns the fixed rotation.
   *
   * \return  The fixed rotation.
   */
    const TMatrix3x3 & getFixedRotation() const;

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
   * \param [out] jacobian  The jacobian.
   * \param referencePoint  The reference point.
   */
    virtual void makeJacobian(TMatrix6xX & /*jacobian*/, const TVector3 & /*referencePoint*/) const;

    /**
   * \brief Integrate one timestep.
   *
   * \param dt  The timestep length to integrate over.
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
    virtual ~FixedRotation();

protected:
    /**
   * \brief Calculate relative pose of this element.
   *
   * \param relr  The relative position.
   * \param relR  The relative rotation.
   */
    virtual void calcRelPose(TVector3 & /*relr*/, TMatrix3x3 & relR);

    /// The fixed relative rotation
    TMatrix3x3 fixedRelR;

    EMPTY_PARAMETER_LIST

}; // class FixedRotation

} // namespace mbslib

#endif
