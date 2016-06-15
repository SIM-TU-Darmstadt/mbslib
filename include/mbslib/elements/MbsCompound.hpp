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
* \file mbslib/elements/MbsCompound.hpp
* Declaration of mbslib::MbsCompound
*/
#ifndef __MBSLIB_MBSCOMPOUND_HPP__
#define __MBSLIB_MBSCOMPOUND_HPP__

#include <mbslib/parameter/ParametrizedObjectCollection.hpp>

#include <mbslib/elements/MbsObject.hpp>
#include <mbslib/elements/UserStateVariable.hpp>

#include <mbslib/elements/base/Base.hpp>
#include <mbslib/elements/endpoint/Endpoint.hpp>
#include <mbslib/elements/joint/Joint.hpp>

#include <mbslib/elements/drive/Drive.hpp>
#include <mbslib/elements/drive/ActiveDrive.hpp>

#include <mbslib/elements/force/ForceGeneratorSet.hpp>

#include <mbslib/collision/PointContact.hpp>

#include <mbslib/elements/Controllable.hpp>

#include <vector>

namespace mbslib {

/**
  * \brief Model of a tree shaped mbs.
  *
  * This class encapsulates the mbs, allows access to all of it joints via state vectors and provides algorithms for calculation of the kinematics and dynamics as well as for time integration.
  */
class MbsCompound : public ParametrizedObjectCollection

{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
      * \brief Constructor.
      */
    MbsCompound(const std::string & name = "");

    /**
    * \brief Destructor.
    */
    virtual ~MbsCompound();

    /**
    * \brief Query if this object is valid.
    *
    * \return  true if valid, false if not.
    */
    virtual bool isValid() const;

    /**
    * \brief Query if this object has base.
    *
    * \return  true if base, false if not.
    */
    bool hasBase() const;

    /**
    * \brief Gets the base.
    *
    * \return  The base.
    */
    Base & getBase();

    /**
    * \brief Gets the base.
    *
    * \return  The base.
    */
    const Base & getBase() const;

    /**
    * \brief Gets the end of MBS.
    *
    * \return  The end.
    */

    Endpoint & getEnd();

    /**
    * \brief Gets the end of MBS.
    *
    * \return  The end.
    */
    const Endpoint & getEnd() const;

    /**
    * \brief Gets an end of MBS.
    *
    * \param i Zero-based index of the Endpoint.
    *
    * \return  The i-th endpoint.
    */
    Endpoint & getEnd(size_t i);

    /**
    * \brief Gets an end of MBS.
    *
    * \param i Zero-based index of the Endpoint.
    *
    * \return  The i-th endpoint.
    */
    const Endpoint & getEnd(size_t i) const;

    const std::vector< Endpoint * > & getEndpoints() const;

    /**
    * \brief Gets the number of endpoints.
    *
    * \return  The number of endpoints.
    */
    size_t getNumberOfEndpoints() const;

    /**
    * \brief Calculates the Direct Kinematics
    */

    void doDirkin();

    /**
    * \brief Calculated RNE.
    *
    * This calculation will use the gravitation vector set for the mbs.
    * In the default mode, external forces will be recalculated. If dontRecalcExternalForces == true, only currently set external forces will be used.
    *
    * \param dontRecalcExternalForces if this is set to true, external forces will not be calculated, but the currently set values will be used.
    */
    void doRne(bool dontRecalcExternalForces = false);

    /**
    * \brief Calculated RNE.
    *
    * This calculation will use an arbitrary vector of gravitation.
    * In the default mode, external forces will be recalculated. If dontRecalcExternalForces == true, only currently set external forces will be used.
    *
    * \param g The vector of gravitation to use for the calculation.
    * \param dontRecalcExternalForces if this is set to true, external forces will not be calculated, but the currently set values will be used.
    */
    void doRne(const TVector3 & g, bool dontRecalcExternalForces = false);

    /**
    * \brief Calculate forward dynamics using CRBA.
    */
    void doCrba();

    /**
    * \brief Calculate forward dynamics using CRBA.
    */
    void doCrba(const TVector3 & g);

    /**
    * \brief Calculate forward dynamics using CRBA.
    */
    void doCrba(const std::vector< PointContact > & contacts);

    /**
    * \brief Calculate forward dynamics using CRBA.
    */
    void doCrba(const TVector3 & g, const std::vector< PointContact > & contacts);

    /**
    * \brief Calculate forward dynamics using ABA.
    *
    * This version of the algorithm will use the gravitation set for the mbs.
    * In the default mode, external forces will be recalculated. If dontRecalcExternalForces == true, only currently set external forces will be used.
    *
    * \param dontRecalcExternalForces if this is set to true, external forces will not be calculated, but the currently set values will be used.
    */
    void doABA(bool dontRecalcExternalForces = false);

    /**
    * \brief Calculate forward dynamics using ABA.
    *
    * This version of the algorithm will use an arbitrary gravitation vector.
    * In the default mode, external forces will be recalculated. If dontRecalcExternalForces == true, only currently set external forces will be used.
    *
    * \param g The vector of gravitation to use for the calculation.
    * \param dontRecalcExternalForces if this is set to true, external forces will not be calculated, but the currently set values will be used.
    */
    void doABA(const TVector3 & g, bool dontRecalcExternalForces = false);

    /**
    * \brief Calculate inverse dynamics ABA.
    *
    * This version of the algorithm will use the gravitation set for the mbs.
    * In the default mode, external forces will be recalculated. If dontRecalcExternalForces == true, only currently set external forces will be used.
    *
    * \param dontRecalcExternalForces if this is set to true, external forces will not be calculated, but the currently set values will be used.
    */
    void doABAinv(bool dontRecalcExternalForces = false);

    /**
    * \brief Calculate inverse dynamics ABA.
    *
    * This version of the algorithm will use an arbitrary gravitation vector.
    * In the default mode, external forces will be recalculated. If dontRecalcExternalForces == true, only currently set external forces will be used.
    *
    * \param g The vector of gravitation to use for the calculation.
    * \param dontRecalcExternalForces if this is set to true, external forces will not be calculated, but the currently set values will be used.
    */
    void doABAinv(const TVector3 & g, bool dontRecalcExternalForces = false);

    /**
    * Calculate hybrid dynamics ABA.
    *
    * This version of the algorithm will use the gravitation set for the mbs.
    * In the default mode, external forces will be recalculated. If dontRecalcExternalForces == true, only currently set external forces will be used.
    *
    * \param doDirectDyn A vector of bool with one element for each joint. True == do direct dynamics for this joint.
    * \param dontRecalcExternalForces if this is set to true, external forces will not be calculated, but the currently set values will be used.
    */
    void doABAhyb(const std::vector< bool > & doDirectDyn, bool dontRecalcExternalForces = false);

    /**
    * Calculate hybrid dynamics ABA.
    *
    * This version of the algorithm will use an arbitrary gravitation vector.
    * In the default mode, external forces will be recalculated. If dontRecalcExternalForces == true, only currently set external forces will be used.
    *
    * \param g The vector of gravitation to use for the calculation.
    * \param doDirectDyn A vector of bool with one element for each joint. True == do direct dynamics for this joint.
    * \param dontRecalcExternalForces if this is set to true, external forces will not be calculated, but the currently set values will be used.
    */
    void doABAhyb(const TVector3 & g, const std::vector< bool > & doDirectDyn, bool dontRecalcExternalForces = false);

    /**
    * \brief Calculate positon part of direct kinematics.
    */
    void doPosition();

    /**
    * \brief Calculate velocity part of direct kinematics.
    */
    void doVelocity();

    /**
    * \brief Calculate acceleration part of direct kinematics.
    */
    void doAcceleration();

    /**
    * \brief Calculate 2nd sweep of Inverse Dynamics.
    */
    void doRneInward(bool withExternalForce);

    /**
    * \brief Calculate aggregate bodies of all subtrees.
    */
    void doAggregateBody();

    /**
    * \brief Calculate mass matrix.
    *
    *  This is a simple method for calculation of the mass matrix based on repetitive evaluation of
    *  the RNE. The algorithm has O(n**2) complexity.
    *
    * \return  The calculated mass matrix.
    */
    TMatrixX calculateMassMatrix();

    /**
    * \brief Make jacobian of a given object's motion.
    *
    * \param o       \todo Kommentieren.
    * \param relPos  (optional) the relative position.
    *
    * \return  The calculated jacobian.
    */

    TMatrix6xX calculateJacobian(const MbsObject & o, const TVector3 & relPos = TVector3::Zero());

    /**
    * \brief Calculate mass matrix.
    *
    *  This is an advanced method for the calculation of the mass matrix based on the CRBA. The
    *  algorithm has O(n**2) complexity, but a better performance than calculateMassMatrix().
    *
    * \return  The calculated mass matrix.
    */
    TMatrixX calculateMassMatrix2();

    /**
    * \brief Calculate coriolis vector.
    *
    * \return  The calculated coriolis force in the joints.
    */
    TVectorX calculateCoriolisForceInJoints();

    /**
    * \brief Calculates the gravitation vector in joints.
    *
    * \return  The calculated gravitation vector in joints.
    */
    TVectorX calculateGravitationVectorInJoints();

    /**
    * \brief Calculate gravitation vector in joints unsing an arbitrary gravitation vector.
    *
    * \param g The g.
    *
    * \return  The calculated gravitation vector in joints.
    */
    TVectorX calculateGravitationVectorInJoints(const TVector3 & g);

    /**
    * \brief Calculate result of external forces/toque in joint.
    *
    * \return  The calculated external force torque in joints.
    */
    TVectorX calculateExternalForceTorqueInJoints();

    /**
    * \brief Integrate one timestep.
    *
    * \param dt  The timestep length.
    */
    void integrate(TTime dt);

    /**
    * \brief Store current state.
    */
    void storeState();

    /**
    * \brief Go back to last stored state.
    */
    void restoreState();

    /**
    * \brief Get number of state variables.
    *
    * \return  The number of state variables.
    */
    size_t getNumberOfStateVariables() const;

    /**
    * \brief Get state.
    *
    * \return  The state.
    */
    TVectorX getState() const;

    /** IDynamicalSystem::getState
    */
    void getState(TVectorX & x) const;

    /**
    * \brief Get first derivative of state w.r.t. to time.
    *
    * \return  The first derivative if the state w.r.t. dt.
    */
    TVectorX getDStateDt() const;

    /**
    * \brief Set state.
    *
    * \param state The state.
    */

    void setState(const TVectorX & state);

    /**
    * \brief Get number of joints.
    *
    * \return  The number of joints.
    */
    size_t getNumberOfJoints() const;

    /**
    * \brief Get joints.
    *
    * \return  The vector if joints in this MBS
    */
    std::vector< Joint1DOF * > & getJoints();

    /**
    * \brief Get joints.
    *
    * \return   The vector of joints in this MBS
    */
    const std::vector< Joint1DOF * > & getJoints() const;

    /**
    * \brief Get DOF of compound.
    *
    * \return  The degrees of freedom.
    */
    int getDOF() const;

    /**
    * \brief Set position of joints.
    *
    * \param q The joint positions.
    */

    void setJointPosition(const TVectorX & q);

    /**
    * \brief Set velocity of joints.
    *
    * \param dq  The joint velocities.
    */
    void setJointVelocity(const TVectorX & dq);

    /**
    * \brief Set acceleration of joints.
    *
    * \param ddq The joint accelerations.
    */
    void setJointAcceleration(const TVectorX & ddq);

    /**
    * \brief Set force of joints.
    *
    * \param tau The joint forces/torques.
    */
    void setJointForceTorque(const TVectorX & tau);

    /**
    * \brief Get position of joints.
    *
    * \return  The joint positions.
    */
    TVectorX getJointPosition();

    /**
    * \brief Get velocity of joints.
    *
    * \return  The joint velocities.
    */
    TVectorX getJointVelocity();

    /**
    * \brief Get acceleration of joints.
    *
    * \return  The joint accelerations.
    */
    TVectorX getJointAcceleration();

    /**
    * \brief Get force of joints.
    *
    * \return  The joint forces/torques.
    */
    TVectorX getJointForceTorque();

    /**
    * \brief Get vector of gravitation.
    *
    * \return  The gravitation.
    */
    const TVector3 & getGravitation() const;

    /**
    * \brief Set vector of gravitation.
    *
    * \param g The desired gravitation.
    */
    void setGravitation(const TVector3 & g);

    /**
    * \brief Get number of active drives.
    *
    * \return  The number of active drives.
    */
    size_t getNumberOfActiveDrives() const;

    /**
    * \brief Set position of joints.
    *
    * \param q The positions.
    */
    void setActiveDrivePosition(const TVectorX & q);

    /**
    * \brief Set velocity of joints.
    *
    * \param dq  The velocities.
    */
    void setActiveDriveVelocity(const TVectorX & dq);

    /**
    * \brief Calculate forces generated by all drives (active or passive).
    */
    void doForwardDrives();

    /**
    * \brief Calculate joint forces generated by all drives (active or passive).
    */
    void doInverseDrives();

    /**
    * \brief Get number of controllers of type Controllable.
    *
    * \return  Number of controllers.
    */
    size_t getNumberOfControllables() const;

    /**
    * \brief Get a Controllable.
    *
    * \param i Zero-based index of the Controllable.
    *
    * \return  The controllable.
    */
    const Controllable & getControllable(size_t i) const;

    /**
    * \brief Get number of control values.
    *
    * \return  The number of control values.
    */
    unsigned int getNumberOfControlValues() const;

    /**
    * \brief Set control values.
    *
    * \param u The control values.
    */
    void setControlValues(const TVectorX & u);

    /**
    * \brief Get control values.
    *
    * \return  The control values.
    */
    TVectorX getControlValues() const;

    bool getIsStateVariableUnconstrainedAngle(unsigned int i) const;

    std::string getControlValueName(unsigned int i) const;

    void getControlLimits(TVectorX & lower, TVectorX & upper) const;
    void getStateLimits(TVectorX & lower, TVectorX & upper) const;

    /**
    * \brief Calculate the sensitivity of tau to the control values.
    *
    * \return  The calculated sensitivity matrix of tau.
    */
    TMatrixX calculateDtauDcontrol();

    /**
    * \brief Calculate the sensitivity of ddq to the control values.
    *
    * \return  The calculated sensitivity matrix of the accelerations.
    */
    TMatrixX calculateDddqDcontrol();

    /**
    * \brief Calculate the sensitivity of ddq to the control values.
    *
    * \return  The calculated sensititvity matrix.
    */
    TMatrixX calculateDddqDcontrol2();

    const std::vector< MbsObject * > & getElements() const;

    /**
     * @brief Get element by its name.
     *
     * @param [in] name Element name
     * @return Pointer to object with specified name. nullptr if it fails.
     */
    MbsObject * getElementByName(const std::string & name) const;
    Endpoint * getEndpointByName(const std::string & name) const;
    Joint1DOF * getJointByName(const std::string & name) const;

    const ForceGeneratorSet & getForceGeneratorSet() const;
    const std::vector< ActiveDrive * > & getActiveDrives() const;

    virtual std::string getStateVariableName(unsigned int i) const;

    void addUserStateVariable(UserStateVariable & variable);

    /**
    * \brief Add an element.
    *
    * \param element  the element.
    */
    void addElement(MbsObject * element);

    /**
    * \brief Add a drive.
    *
    * \param drive If non-null, the drive.
    */

    void addDrive(Drive * drive);

    /**
    * \brief Add force generator.
    *
    * \param fg  If non-null, the forcegenerator.
    */
    virtual void addForceGenerator(ForceGenerator * fg);

private:
    /// Elements of the MBS.
    std::vector< MbsObject * > elements;

    //protected:
    /// Joints of the MBS.
    std::vector< Joint1DOF * > joints;

    /// sum of DOF of all joints
    int dof;

    /// Endpoints of the MBS.
    std::vector< Endpoint * > endpoints;

    /// Base of the MBS.
    Base * base;

    /// Type of base
    bool freeBase;

    /// Last endpoint of MBS.
    Endpoint * endpoint;

    /// Objects which need integration.
    std::vector< IIntegrate * > objectsWithIntegrator;

    /// Subset of statevector represented by joints.
    std::vector< IIntegrate * > stateVectorJoints;

    /// Size of state vector caused by joints
    size_t statesOfJoints;

    /// Subset of statevector represented by other objects.
    std::vector< IIntegrate * > stateVectorNotJoints;

    /// Size of state vector caused by other objects.
    size_t statesOfNonJoints;

    /// Objects which can be controlled by a control value
    std::vector< Controllable * > controllables;

    /// UserStateVariables
    std::vector< UserStateVariable * > userStateVariables;

    /// Sum of control values.
    unsigned int numberOfControlValues;

    /// Drives.
    std::vector< Drive * > drives;

    /// Active drives.
    std::vector< ActiveDrive * > activeDrives;

    /// Vector of gravitation.
    TVector3 gravitationVector;

    /// Force generators.
    ForceGeneratorSet forceGeneratorSet;

}; // class MbsCompound

} // namespace mbslib

#endif
