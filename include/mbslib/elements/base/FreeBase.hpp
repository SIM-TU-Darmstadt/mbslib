/**
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
 * The MBSlib is distributed WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with MBSlib.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \file mbslib/elements/base/FreeBase.hpp
 * Declaration of mbslib::FreeBase
 */
#ifndef __MBSLIB_FREEBASE_HPP__
#define __MBSLIB_FREEBASE_HPP__

#include <mbslib/elements/base/Base.hpp>

namespace mbslib {

/**
 * \brief Free base.
 *
 * Representation of a mbs which can move as a result of the mbs' motion or external forces.
 */
class FreeBase : public Base, public IIntegrate {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
   * \brief Constructor.
   *
   * \param name  (optional) the name.
   */
    FreeBase(const std::string & name = "");

    /**
   * \brief Constructor.
   *
   * \param position    The position.
   * \param orientation The orientation.
   * \param name        (optional) the name.
   */
    FreeBase(const TVector3 & position, const TMatrix3x3 & orientation, const std::string & name = "");

    virtual ~FreeBase();

    /**
   * \brief Get DOF of element.
   *
   * \return  The degree of freedom.
   */
    virtual int getDOF() const;

    /**
   * \brief Calculated positon part of direct kinematics.
   */
    virtual void doPosition();

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
   * \param withExternalForce true to take into account external force.
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
    virtual void makeJacobian(TMatrix6xX & jacobian, const TVector3 & referencePoint) const;

    /**
   * \brief Integrate one timestep.
   *
   * \param dt  The timestep to integrate over.
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
   * 	\warning Currently there is no way to access the state of
   *  the FreeBase externally!
   *
   * \return  The number of state variables.
   */
    virtual size_t getNumberOfStateVariables() const;

    /**
   * \brief Get state. 
   * 				
   * 	\warning Currently there is no way to access the state of the FreeBase
   *  externally!
   *
   * \return  The state vector.
   */
    virtual TVectorX getState() const;

    /**
   * \brief Get first derivative of state w.r.t. to time. \warning Currently there is no way to
   *  access the state of the FreeBase externally! \todo kommentieren
   *
   * \return  The d state dt.
   */
    virtual TVectorX getDStateDt() const;

    /**
   * \brief Set state. \warning Currently there is no way to access the state of the FreeBase
   *  externally! \todo kommentieren
   *
   * \param   The state vector.
   */
    virtual void setState(const TVectorX & state);

protected:
    /**
   * \brief Calculate relative pose of this element.
   *
   * \param relr  The relative position.
   * \param relR  The relative rotation.
   */
    virtual void calcRelPose(TVector3 & relr, TMatrix3x3 & relR);

    /// temporary value for ABA, calcualted in sweep one, used in sweeps two and three
    TVector6 c;

    /// temporary value for ABA, calcualted in sweep two, used in sweep  three
    TMatrix6x6 U;

    /// temporary value for ABA, calcualted in sweep two, used in sweep  three
    TMatrix6x6 D;

    /// temporary value for ABA, calcualted in sweep two, used in sweep  three
    TVector6 u;

    /// stored coordinate frame for integrator-go-backward-functionality
    CoordinateFrame storedCof;

    EMPTY_PARAMETER_LIST

}; // class FreeBase

} // namespace mbslib

#endif
