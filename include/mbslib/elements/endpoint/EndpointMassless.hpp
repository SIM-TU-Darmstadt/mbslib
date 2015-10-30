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
 * \file mbslib/elements/endpoint/EndpointMassless.h
 * Declaration of mbslib::EndpointMassless
 */
#ifndef __MBSLIB_ENDPOINTMASSLESS_HPP__
#define __MBSLIB_ENDPOINTMASSLESS_HPP__

#include <mbslib/elements/endpoint/Endpoint.hpp>

namespace mbslib {

/**
 * \brief Massless Endpoint. \todo kommentieren
 *
 * Representation of an Endpoint of the mbs. This is an Endpoint which has neither mass nor inertia.
 */
class EndpointMassless
    : public Endpoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
   * \brief Constructor.
   *
   * \param pred  The predecessor.
   * \param name  (optional) the name.
   */
    EndpointMassless(MbsObject & pred, const std::string & name = "");

    /**
   * \brief Get DOF of element.
   *
   * \return  The degrees of freedom.
   */
    virtual int getDOF() const {
        return 0;
    }

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
   * \param jacobian        The jacobian.
   * \param referencePoint  The reference point.
   */
    virtual void makeJacobian(TMatrix6xX & /*jacobian*/, const TVector3 & /*referencePoint*/) const;

    /**
   * \brief Add external force/torque.
   *
   * \param f The force.
   * \param t The torque.
   */
    virtual void addExternalForceTorque(const TVector3 & f, const TVector3 & t);

    /**
   * \brief Add external force/torque given in world coordinates.
   *
   * \param f The force.
   * \param t The torque.
   */
    virtual void addExternalForceTorqueWCS(const TVector3 & f, const TVector3 & t);

    /**
   * \brief Set external force/torque.
   *
   * \param f The force.
   * \param t The torque.
   */
    virtual void setExternalForceTorque(const TVector3 & f, const TVector3 & t);

    /**
   * \brief Set external force/torque given in world coordinates.
   *
   * \param f The force.
   * \param t The torque.
   */
    virtual void setExternalForceTorqueWCS(const TVector3 & f, const TVector3 & t);

    /**
   * \brief Get external force.
   *
   * \return  The external force.
   */
    virtual const TVector3 & getExternalForce() const;

    /**
   * \brief Get external torque.
   *
   * \return  The external torque.
   */
    virtual const TVector3 & getExternalTorque() const;

    /**
   * \brief Integrate one timestep.
   *
   * \param dt  The timestep to integrate over.
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
    virtual ~EndpointMassless();

protected:
    /**
   * \brief Calculate relative pose of this element.
   *
   * \param relr  The relative position.
   * \param relR  The relative rotation.
   */
    virtual void calcRelPose(TVector3 & /*relr*/, TMatrix3x3 & /*relR*/) {
    }

    /// External force in local coordinates of EndpointMassless.
    TVector3 extForce;

    /// External torque in local coordinates of EndpointMassless.
    TVector3 extTorque;

    EMPTY_PARAMETER_LIST
}; // class EndpointMassless

} // namespace mbslib

#endif
