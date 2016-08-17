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
 * \file mbslib/elements/compound/MbsCompoundWithBuilder.hpp
 * Declaration of mbslib::MbsCompoundWithBuilder
 */
#ifndef __MBSLIB_MBSCOMPOUNDWITHBUILDER_HPP__
#define __MBSLIB_MBSCOMPOUNDWITHBUILDER_HPP__

#include <mbslib/elements/MbsCompound.hpp>

#include <mbslib/elements/base/FixedBase.hpp>
#include <mbslib/elements/base/FreeBase.hpp>
#include <mbslib/elements/drive/Drive.hpp>
#include <mbslib/elements/drive/DriveGenerator.hpp>
#include <mbslib/elements/endpoint/EndpointMassless.hpp>
#include <mbslib/elements/fixed/FixedRotation.hpp>
#include <mbslib/elements/fixed/FixedTranslation.hpp>
#include <mbslib/elements/fork/Fork.hpp>
#include <mbslib/elements/joint/PrismaticJoint.hpp>
#include <mbslib/elements/joint/RevoluteJoint.hpp>
#include <mbslib/elements/joint/RevoluteJointZ.hpp>
#include <mbslib/elements/muscle/Muscle.hpp>
#include <mbslib/elements/rigidbody/RigidLink.hpp>
#include <mbslib/elements/spring/Spring1D.hpp>
#include <mbslib/elements/spring/Spring1DSingleEnded.hpp>
#include <mbslib/elements/spring/Spring3D.hpp>

#include <map>
#include <stack>

namespace mbslib {

/**
 * \brief Mbs compound with builder.
 */
class MbsCompoundWithBuilder : public MbsCompound {
public:
    /**
   * \brief Constructor.
   *
   * \param name  (optional) the name.
   */
    MbsCompoundWithBuilder(const std::string & name = "");

    /**
   * \brief Destructor.
   */
    virtual ~MbsCompoundWithBuilder();

    /**
   * \brief Adds a fixed base.
   *
   * \param name  (optional) the name.
   *
   * \return  null if it fails, else the base.
   */
    FixedBase * addFixedBase(const std::string & name = "");

    /**
   * \brief Adds a fixed base.
   *
   * \param position    The position.
   * \param orientation The orientation.
   * \param name        (optional) the name.
   *
   * \return  null if it fails, else the base.
   */

    FixedBase * addFixedBase(const TVector3 & position, const TMatrix3x3 & orientation, const std::string & name = "");

    /**
   * \brief Adds a free base.
   *
   * \param name  (optional) the name.
   *
   * \return  null if it fails, else the base.
   */
    FreeBase * addFreeBase(const std::string & name = "");

    /**
   * \brief Adds a free base.
   *
   * \param position    The position.
   * \param orientation The orientation.
   * \param name        (optional) the name.
   *
   * \return  null if it fails, else the base.
   */
    FreeBase * addFreeBase(const TVector3 & position, const TMatrix3x3 & orientation, const std::string & name = "");

    /**
   * \brief Adds an massless endpoint.
   *
   * \param name  (optional) the name.
   *
   * \return  null if it fails, else the endpoint.
   */
    EndpointMassless * addEndpointMassless(const std::string & name = "");

    /**
   * \brief Adds an endpoint.
   *
   * \param name  (optional) the name.
   *
   * \return  null if it fails, else the endpoint.
   */
    Endpoint * addEndpoint(const std::string & name = "");

    /**
   * \brief Adds an endpoint.
   *
   * \param m     The mass.
   * \param I     The inertia tensor.
   * \param name  (optional) the name.
   *
   * \return  null if it fails, else the endpoint.
   */
    Endpoint * addEndpoint(TScalar m, const TMatrix3x3 & I, const std::string & name = "");

    /**
   * \brief Adds a fixed rotation.
   *
   * \param relR  The relative rotation to the previous element.
   * \param name  (optional) the name.
   *
   * \return  null if it fails, else the rotation.
   */
    FixedRotation * addFixedRotation(const TMatrix3x3 & relR, const std::string & name = "");

    /**
   * \brief Adds a fixed translation.
   *
   * \param relr  The relative position to the previous element.
   * \param name  (optional) the name.
   *
   * \return  null if it fails, else the translation.
   */
    FixedTranslation * addFixedTranslation(const TVector3 & relr, const std::string & name = "");

    /**
   * \brief Adds a prismatic joint.
   *
   * \param dir   The direction of the joint.
   * \param name  The name.
   *
   * \return  null if it fails, else the prismatic joint.
   */
    PrismaticJoint * addPrismaticJoint(const TVector3 & dir, const std::string & name);

    /**
   * \brief Adds a prismatic joint.
   *
   * \param dir         The direction of the joint.
   * \param jointOffset (optional) the joint offset.
   * \param name        (optional) the name.
   *
   * \return  null if it fails, else prismatic joint.
   */
    PrismaticJoint * addPrismaticJoint(const TVector3 & dir, TScalar jointOffset = 0, const std::string & name = "");

    /**
   * \brief Adds a prismatic joint.
   *
   * \param dir           The direction of the joint.
   * \param jointOffset   The joint offset.
   * \param jointFriction The joint friction.
   * \param gearRatio     The gear ratio.
   * \param rotorInertia  The rotor inertia.
   * \param name          (optional) the name.
   *
   * \return  null if it fails, else the prismatic joint.
   */
    PrismaticJoint * addPrismaticJoint(const TVector3 & dir, TScalar jointOffset, TScalar jointFriction, TScalar gearRatio, TScalar rotorInertia, const std::string & name = "");

    /**
   * \brief Adds a revolute joint to 'name'.
   *
   * \param axis  The axis of rotation.
   * \param name  The name.
   *
   * \return  null if it fails, else the revolute joint.
   */
    RevoluteJoint * addRevoluteJoint(const TVector3 & axis, const std::string & name);

    /**
   * \brief Adds a revolute joint.
   *
   * \param axis        The axis of rotation.
   * \param jointOffset (optional) the joint offset.
   * \param name        (optional) the name.
   *
   * \return  null if it fails, else the revolute joint.
   */
    RevoluteJoint * addRevoluteJoint(const TVector3 & axis, TScalar jointOffset = 0, const std::string & name = "");

    /**
   * \brief Adds a revolute joint.
   *
   * \param axis          The axis of rotation.
   * \param jointOffset   The joint offset.
   * \param jointFriction The joint friction.
   * \param gearRatio     The gear ratio.
   * \param rotorInertia  The rotor inertia.
   * \param name          (optional) the name.
   *
   * \return  null if it fails, else the revolute joint.
   */
    RevoluteJoint * addRevoluteJoint(const TVector3 & axis, TScalar jointOffset, TScalar jointFriction, TScalar gearRatio, TScalar rotorInertia, const std::string & name = "");

    /**
   * \brief Add revolute joint around z axis.
   *
   * \param jointOffset (optional) the joint offset.
   * \param name        (optional) the name.
   *
   * \return  null if it fails, else the revolute joint.
   */

    RevoluteJointZ * addRevoluteJointZ(TScalar jointOffset = 0, const std::string & name = "");

    /**
   * \brief Add revolute joint around z axis.
   *
   * \param jointOffset   The joint offset.
   * \param jointFriction The joint friction.
   * \param gearRatio     The gear ratio.
   * \param rotorInertia  The rotor inertia.
   * \param name          (optional) the name.
   *
   * \return  null if it fails, else the revolute joint.
   */

    RevoluteJointZ * addRevoluteJointZ(TScalar jointOffset, TScalar jointFriction, TScalar gearRatio, TScalar rotorInertia, const std::string & name = "");

    /**
   * \brief Add rigid link.
   *
   * \param relr  The relative position.
   * \param com   The com.\todo kommentieren
   * \param m     The mass.
   * \param I     The inertia tensor.
   * \param name  (optional) the name.
   *
   * \return  null if it fails, else the rigid link.
   */
    RigidLink * addRigidLink(const TVector3 & relr, const TVector3 & com, TScalar m, const TMatrix3x3 & I, const std::string & name = "");

    /**
   * \brief Add rigid link.
   *
   * \param rigidBody The rigid body.
   * \param name      (optional) the name.
   *
   * \return  null if it fails, else rigid link.
   */
    RigidLink * addRigidLink(const RigidBodyDescription & rigidBody, const std::string & name = "");

    /**
   * \brief Add a fork.
   *
   * \param name  (optional) the name.
   *
   * \return  null if it fails, else the fork.
   */
    Fork * addFork(const std::string & name = "");

    /**
   * \brief Add a drive. This will only work if the last element was a joint and has no drive yet.
   *
   * \param driveGenerator  The drive generator.
   * \param name            (optional) the name.
   *
   * \return  null if it fails, else the drive.
   */
    Drive * addDrive(const DriveGenerator & driveGenerator, const std::string & name = "");

    /**
   * \brief Add a drive.
   *
   * \param driveGenerator  The drive generator.
   * \param joint           The joint.
   * \param name            (optional) the name.
   *
   * \return  null if it fails, else the drive.
   */
    Drive * addDrive(const DriveGenerator & driveGenerator, Joint1DOF & joint, const std::string & name = "");

    /**
   * \brief Add Spring.
   *
   * \param model The spring model.
   * \param name  (optional) the name.
   *
   * \return  null if it fails, else the spring.
   */
    Spring3D * addSpring(const SpringModel & model, const std::string & name = "");

    /**
   * \brief Add Spring.
   *
   * \param model             The spring model.
   * \param points            If non-null, the points. \todo kommentieren
   * \param name              (optional) the name.
   *
   * \return  null if it fails, else the spring.
   */
    Spring3D * addSpring(const SpringModel & model, std::vector< Endpoint * > points, const std::string & name = "");

    /**
   * \brief Add Spring.
   *
   * \param model    The springmodel.
   * \param j1       The first Joint1DOF &. \todo kommentieren
   * \param j2       The second Joint1DOF &. \todo kommentieren
   * \param name     (optional) the name.
   *
   * \return  null if it fails, else the spring.
   */
    Spring1D * addSpring(const SpringModel & model, Joint1DOF & j1, Joint1DOF & j2, const std::string & name = "");

    /**
   * \brief Add Spring.
   *
   * \param model    The springmodel.
   * \param j1       The Joint1DOF &. \todo kommentieren
   * \param name     (optional) the name.
   *
   * \return  null if it fails, else the spring.
   */
    Spring1DSingleEnded * addSpring(const SpringModel & model, Joint1DOF & j1, const std::string & name = "");

    /**
   * \brief Add muscle.
   *
   * \param model The muscle model.
   * \param name  (optional) the name.
   *
   * \return  null if it fails, else the muscle.
   */
    Muscle * addMuscle(const MuscleModel & model, const std::string & name = "");

    /**
     * @brief Adds joint and link according to given Denavit Hartenberg (DH) description.
     *
     * @param jointType     Joint type, prismatic or revolute
     * @param theta         Parameter theta in DH description
     * @param d             Parameter d in DH description
     * @param a             Parameter a in DH description
     * @param alpha         Parameter alpha in DH description
     * @param com           Vector to center of mass relative to link coordinate frame
     * @param m             Link mass
     * @param I             Link inertia tensor according to center of mass and link coordinate system.
     * @param jointName     Joint name
     * @param linkName      Link name
     * @return empty vector if it fails, else
     */
    std::vector< MbsObject * > addDHJointLink(JointType jointType, TScalar theta, TScalar d, TScalar a, TScalar alpha, const TVector3 & com, TScalar m, const TMatrix3x3 & I, const std::string & jointName = "", const std::string & linkName = "");

    /**
     * @brief Adds joint and link according to given Denavit Hartenberg (DH) description.
     *
     * @param jointType     Joint type, prismatic or revolute
     * @param theta         Parameter theta in DH description
     * @param d             Parameter d in DH description
     * @param a             Parameter a in DH description
     * @param alpha         Parameter alpha in DH description
     * @param com           Vector to center of mass relative to link coordinate frame
     * @param m             Link mass
     * @param I             Link inertia tensor according to center of mass and link coordinate system.
     * @param jointName     Joint name
     * @param linkName      Link name
     * @return empty vector if it fails, else
     */
    std::vector< MbsObject * > addDHJointLink2(JointType jointType, TScalar theta, TScalar d, TScalar a, TScalar alpha, const TVector3 & com, TScalar m, const TMatrix3x3 & I, const std::string & jointName = "", const std::string & linkName = "");

    /**
   * \brief Get number of muscles.
   *
   * \return  The number of muscles.
   */
    size_t getNumberOfMuscles(void) const;

    /**
   * \brief Get muscles.
   *
   * \return  the vector of muscles.
   */
    std::vector< Muscle * > & getMuscles(void);

    /**
   * \brief Get muscles.
   *
   * \return  the vector of muscles
   */
    const std::vector< Muscle * > & getMuscles(void) const;

    /**
   * \brief Set control value of muscles.
   *
   * \param c The control vector.
   */
    void setMuscleControl(TVectorX c);

    /**
   * \brief Get control value of muscles.
   *
   * \return  The muscle control vector.
   */
    TVectorX getMuscleControl(void) const;

    /**
   * \brief Calculate sensitivity of joint force/torque by muscle-controls.
   *
   * \return  The d tau d muscle control.
   */
    //TMatrixX getDTauDMuscleControl();

    /**
   * \brief Add ext. force generator.
   *
   *  Force-Generators will be destroyed by destructor of this.
   *
   * \param   fg  The force generator.
   */
    virtual void addForceGenerator(ForceGenerator * fg);

protected:
    /**
   * \brief Add element.
   *
   * \param object  the object.
   *
   * \return  null if it fails, else the object.
   */
    MbsObject * addElement(MbsObject * object);

    /// Last added element.
    MbsObject * lastElement;

    /// Chain is still open.
    bool isOpen;

    /// Stack of forks.
    std::stack< Fork * > forkStack;

    /// Last element is a joint and it has no drive yet.
    bool undrivenJoint;

    /// Springs.
    std::vector< Spring * > springs;

    /// Muscles.
    std::vector< Muscle * > muscles;

    /// Force-Generators
    std::vector< ForceGenerator * > forceGenerators;

    std::map< Joint1DOF *, Drive * > jointDriveMap;

}; // class MbsCompoundWithBuilder

} // namespace mbslib

#endif
