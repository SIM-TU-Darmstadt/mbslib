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
 * \file mbslib/elements/MbsObject.hpp
 * Declaration of mbslib ::MbsObject
 */
#ifndef __MBSLIB_MBSOBJECT_HPP__
#define __MBSLIB_MBSOBJECT_HPP__

#include <mbslib/parameter/ParametrizedObject.hpp>
#include <mbslib/elements/IIntegrate.hpp>
#include <mbslib/utility/types.hpp>

#include <string>
#include <vector>

namespace mbslib {

/**
 * \brief  Mbs object. \todo kommentieren
 * 				 CompositeSimulatioObject needs to be initialized first, else the vector it contains will be invalid 
 * 				 in the constructor of MbsObject and its children
 */
class MbsObject : public ParametrizedObject {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
   * \brief Destructor.
   */
    virtual ~MbsObject();

    /**
   * \brief Get predecessor.
   *
   * \return  null if it fails, else the predecessor.
   */
    MbsObject * getPredecessor() {
        return pred;
    }

    /**
   * \brief Get predecessor.
   *
   * \return  null if it fails, else the predecessor.
   */
    const MbsObject * getPredecessor() const {
        return pred;
    }

    /**
   * \brief Gets the successor at index.
   *
   * \param index Zero-based index of the successor.
   *
   * \return  null if it fails, else the successor.
   */
    virtual const MbsObject * getSuccessor(size_t index) const = 0;

    /**
   * \brief Gets the number of successors this MbsObject has.
   *
   * \return  The successor count.
   */
    virtual size_t getSuccessorCount() const = 0;

    /**
   * \brief Check if setup is valid.
   *
   * \return  true if valid, false if not.
   */
    virtual bool isValid() const {
        return !setupError;
    }

    /**
   * \brief Get name of object.
   *
   * \return  The name.
   */
    //const std::string & getName()const { return name; }

    /**
   * \brief Get DOF of element.
   *
   * \return  The degrees of freedom.
   */
    virtual int getDOF() const = 0;

    /**
   * \brief Get coordiante frame of object.
   *
   * \return  The coordinate frame.
   */
    const CoordinateFrame & getCoordinateFrame() const {
        return cof;
    }

    /**
   * \brief Get relative position wrt. predecessor.
   *
   * \return  The relative position.
   */
    const TVector3 & getRelativePosition() const {
        return relr;
    }

    /**
   * \brief Get relative orientation wrt. predecessor.
   *
   * \return  The relative orientation.
   */
    const TMatrix3x3 & getRelativeOrientation() const {
        return relR;
    }

    /**
   * \brief Get force and torque on predecessor.
   *
   * \return  The reverse newton euler force|torque.
   */
    const ForceTorque & getRneForceTorque() const {
        return rneForceTorque;
    }

    /**
   * \brief Get aggregate body of subtree (in coordinates of the predecessors CoF).
   *
   * \return  The aggregate body.
   */
    const AggregateBody & getAggregateBody() const {
        return aggregateBody;
    }

    /**
   * \brief Get Articulated Body Inertia (in the coordinates of the predecessor).
   *
   * \return  The articulated body inertia.
   */
    const TMatrix6x6 & getArticulatedBodyInertia() const {
        return articulatedBodyInertia;
    }

    /**
   * \brief Get bias force (in coordinated of pred.).
   *
   * \return  The bias force.
   */
    const TVector6 & getBiasForce() const {
        return biasForce;
    }

    /**
   * \brief Calculate direct kinematics wrt. to predecessor.
   */
    virtual void doDirkin();

    /**
   * \brief Calculated positon part of direct kinematics.
   */
    virtual void doPosition();

    /**
   * \brief Calculated velocity part of direct kinematics.
   */
    virtual void doVelocity() = 0;

    /**
   * \brief Calculate acceleration part of direct kinematics.
   */
    virtual void doAcceleration() = 0;

    /**
   * \brief Calculate 2nd sweep of Inverse Dynamics.
   *
   * \param withExternalForce (optional) taking into account the external force.
   */
    virtual void doRneInward(bool withExternalForce = false) = 0;

    /**
   * \brief Calculate aggregate body inwards.
   */
    virtual void doAggregateBody() = 0;

    /**
   * \brief First (outward) sweep of forward dynamics ABA.
   */
    virtual void doABASweep1fwd() = 0;

    /**
   * \brief Second (inward) sweep of forward dynamics ABA.
   */
    virtual void doABASweep2fwd() = 0;

    /**
   * \brief Third (outward) sweep of forward dynamics ABA.
   */
    virtual void doABASweep3fwd() = 0;

    /**
   * \brief First (outward) sweep of inverse dynamics ABA.
   */
    virtual void doABASweep1inv() = 0;

    /**
   * \brief Second (inward) sweep of inverse dynamics ABA.
   */
    virtual void doABASweep2inv() = 0;

    /**
   * \brief Third (outward) sweep of inverse dynamics ABA.
   */
    virtual void doABASweep3inv() = 0;

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

    /**
   * \brief Make columns of jacobian for a given position.
   *
   * \param [out]  jacobian  The jacobian.
   * \param referencePoint   The reference point.
   */
    virtual void makeJacobian(TMatrix6xX & jacobian, const TVector3 & referencePoint) const = 0;

    /**
   * \brief Calculate the jacobian recursively.
   *
   * \param [out] jacobian  The jacobian.
   * \param referencePoint  The reference point.
   */
    void calculateJacobian(TMatrix6xX & jacobian, const TVector3 & referencePoint) const;

protected:
    /**
   * \brief Constructor.
   *
   * \param n (optional) the name.
   */
    MbsObject(const std::string & n = "");

    /**
   * \brief Constructor.
   *
   * \param pred  The predecessor.
   * \param n     (optional) the n.
   */
    MbsObject(MbsObject & pred, const std::string & n = "");

    /**
   * \brief Set successor of object.
   *  
   *
   * \param succ  The successor.
   *
   * \return  true if it succeeds, false if it fails.
   */
    virtual bool addSuccessor(MbsObject & succ);

    /**
   * \brief Calculate relative pose of this element.
   *
   * \param relr  The relative position.
   * \param relR  The relative rotation.
   */
    virtual void calcRelPose(TVector3 & relr, TMatrix3x3 & relR) = 0;

    /// name of object
    //std::string name;

    /// predecessor;
    MbsObject * pred;

    /// Coordinate frame associated with object
    CoordinateFrame cof;

    /// Force and torque calculated by RNE.
    ForceTorque rneForceTorque;

    /// Aggregate body of subtree rooting at this element. Used by CRBA
    AggregateBody aggregateBody;

    /// Articulated Body Inertia, used by ABA
    TMatrix6x6 articulatedBodyInertia;

    /// Bias force, used by ABA;
    TVector6 biasForce;

    /// setup error flag.
    bool setupError;

private:
    /// The relative rotation
    TMatrix3x3 relR;

    ///< The rel position
    TVector3 relr;

protected:
    EMPTY_PARAMETER_LIST
}; //class MbsObject

} //namespace mbslib

#endif
