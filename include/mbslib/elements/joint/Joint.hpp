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
 * \file mbslib/elements/joint/Joint.hpp
 * Declaration of mbslib::Joint
 */
#ifndef __MBSLIB_JOINT_HPP__
#define __MBSLIB_JOINT_HPP__

#include <mbslib/elements/object/MbsObjectOneSucc.hpp>

namespace mbslib {

/**
 * \brief Joint.
 *
 * Base class for any kind of joint.
 */
class Joint : public MbsObjectOneSucc, public IIntegrate {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
   * \brief Get DOF of element.
   *
   * \return  The degrees of freedom.
   */
    virtual int getDOF() const = 0;

    /**
   * \brief Get id of this joint in the mbs.
   *
   * \return  The joint identifier.
   */
    int getJointId() const;

    /**
   * \brief Position of this joint's joint-value(s) in the state vector.
   *  
   *  In case there are joints with more than one DOF in the mbs the state-vector-position of any
   *  joint may differ from the joint id.
   *
   * \return  The state vector position.
   */
    int getStateVectorPosition() const;

    /**
   * \brief Get preceeding joint.
   *
   * \return  null if it no joint, else the preceeding joint.
   */
    Joint * getPreceedingJoint();

    /**
   * \brief Integrate one timestep.
   */
    virtual void integrate(TTime dt) = 0;

    /**
   * \brief Store current state.
   */
    virtual void storeState() = 0;

    /**
   * \brief Go back to last stored state.
   */
    virtual void restoreState() = 0;

    /**
   * \brief First (outward) sweep of hybrid dynamics ABA.
   *
   * \param doDirectDyn A vector of bool with one element for each joint. True == do direct dynamics
   *                    for this joint.
   */
    virtual void doABASweep1hyb(const std::vector< bool > & doDirectDyn);

    /**
   * \brief Second (inward) sweep of hybrid dynamics ABA.
   *
   * \param doDirectDyn A vector of bool with one element for each joint. True == do direct dynamics
   *                    for this joint.
   */
    virtual void doABASweep2hyb(const std::vector< bool > & doDirectDyn);

    /**
   * \brief Third (outward) sweep of hybrid dynamics ABA.
   *
   * \param doDirectDyn A vector of bool with one element for each joint. True == do direct dynamics
   *                    for this joint.
   */
    virtual void doABASweep3hyb(const std::vector< bool > & doDirectDyn);
    virtual ~Joint();

protected:
    /**
   * \brief Constructor.
   *
   * \param pred                The predecessor.
   * \param jointId             Identifier for the joint.
   * \param stateVectorPosition The state vector position.
   * \param name                (optional) the name.
   */
    Joint(MbsObject & pred, int jointId, int stateVectorPosition, const std::string & name = "");

    /// id of the joint
    int jointId;

    /// the preceeding joint
    Joint * predJoint;

    /// position of joint's variables in statevector
    int stateVectorPosition;

}; // class Joint

} // namespace mbslib

#endif
