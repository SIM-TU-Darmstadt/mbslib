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
 * \file mbslib/elements/compound/MbsCompoundWithBuilder.cpp
 * Definition of mbslib::MbsCompoundWithBuilder
 */

#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>
#include <Eigen/Geometry>

using namespace mbslib;

std::vector< MbsObject * > operator<<(std::vector< MbsObject * > & vector, MbsObject * obj) {
    vector.push_back(obj);
    return vector;
}

MbsCompoundWithBuilder::MbsCompoundWithBuilder(const std::string & name)
    : MbsCompound(name)
    , lastElement(NULL)
    , isOpen(false)
    , undrivenJoint(false) {
}

MbsCompoundWithBuilder::~MbsCompoundWithBuilder() {
    //for(std::vector<Spring*>::iterator it = springs.begin(); it != springs.end(); it++){ delete (*it); }
    //for(std::vector<Muscle*>::iterator it = muscles.begin(); it != muscles.end(); it++){ delete (*it); }
    //for(std::vector<ForceGenerator*>::iterator it = forceGenerators.begin(); it != forceGenerators.end(); it++){ delete (*it); }
}

FixedBase * MbsCompoundWithBuilder::addFixedBase(const std::string & name) {
    return addFixedBase(TVector3::Zero(), TMatrix3x3::Identity(), name);
}

FixedBase * MbsCompoundWithBuilder::addFixedBase(const TVector3 & position, const TMatrix3x3 & orientation, const std::string & name) {
    if (hasBase()) {
        return NULL;
    }
    FixedBase * fb = new FixedBase(position, orientation, name);
    if (addElement(fb)) {
        isOpen = true;
        return fb;
    } else {
        return NULL;
    }
}

FreeBase * MbsCompoundWithBuilder::addFreeBase(const std::string & name) {
    return addFreeBase(TVector3::Zero(), TMatrix3x3::Identity(), name);
}

FreeBase * MbsCompoundWithBuilder::addFreeBase(const TVector3 & position, const TMatrix3x3 & orientation, const std::string & name) {
    if (hasBase()) {
        return NULL;
    }
    FreeBase * fb = new FreeBase(position, orientation, name);
    if (addElement(fb)) {
        isOpen = true;
        return fb;
    } else {
        return NULL;
    }
}

EndpointMassless * MbsCompoundWithBuilder::addEndpointMassless(const std::string & name) {
    if (!isOpen) {
        return NULL;
    }
    EndpointMassless * ep = new EndpointMassless(*lastElement, name);
    if (!addElement(ep)) {
        return NULL;
    } else {
        while (!forkStack.empty()) {
            if (!(forkStack.top()->isConnected())) {
                lastElement = forkStack.top();
                return ep;
            } else {
                forkStack.pop();
            }
        }
        isOpen = false;
        return ep;
    }
}

Endpoint * MbsCompoundWithBuilder::addEndpoint(const std::string & name) {
    return addEndpoint(static_cast< TScalar >(0), TMatrix3x3::Zero(), name);
}

Endpoint * MbsCompoundWithBuilder::addEndpoint(TScalar m, const TMatrix3x3 & I, const std::string & name) {
    if (!isOpen) {
        return NULL;
    }
    Endpoint * ep = new Endpoint(*lastElement, m, I, name);
    if (!addElement(ep)) {
        return NULL;
    } else {
        while (!forkStack.empty()) {
            if (!(forkStack.top()->isConnected())) {
                lastElement = forkStack.top();
                return ep;
            } else {
                forkStack.pop();
            }
        }
        isOpen = false;
        return ep;
    }
}

Fork * MbsCompoundWithBuilder::addFork(const std::string & name) {
    if (!isOpen) {
        return NULL;
    }
    Fork * f = new Fork(*lastElement, name);
    if (addElement(f)) {
        forkStack.push(f);
        return f;
    } else {
        return NULL;
    }
}

FixedRotation * MbsCompoundWithBuilder::addFixedRotation(const TMatrix3x3 & relR, const std::string & name) {
    if (!isOpen) {
        return NULL;
    }
    FixedRotation * fr = new FixedRotation(*lastElement, relR, name);
    if (addElement(fr)) {
        return fr;
    } else {
        return NULL;
    }
}

FixedTranslation * MbsCompoundWithBuilder::addFixedTranslation(const TVector3 & relr, const std::string & name) {
    if (!isOpen) {
        return NULL;
    }
    FixedTranslation * ft = new FixedTranslation(*lastElement, relr, name);
    if (addElement(ft)) {
        return ft;
    } else {
        return NULL;
    }
}

PrismaticJoint * MbsCompoundWithBuilder::addPrismaticJoint(const TVector3 & dir, const std::string & name) {
    if (!isOpen) {
        return NULL;
    }
    PrismaticJoint * pj = new PrismaticJoint(*lastElement, dir, getNumberOfJoints(), getDOF(), 0, 0, 0, 0, name);
    if (addElement(pj)) {
        return pj;
    } else {
        return NULL;
    }
}

PrismaticJoint * MbsCompoundWithBuilder::addPrismaticJoint(const TVector3 & dir, TScalar jointOffset, const std::string & name) {
    if (!isOpen) {
        return NULL;
    }
    PrismaticJoint * pj = new PrismaticJoint(*lastElement, dir, getNumberOfJoints(), getDOF(), jointOffset, 0, 0, 0, name);
    if (addElement(pj)) {
        return pj;
    } else {
        return NULL;
    }
}

PrismaticJoint * MbsCompoundWithBuilder::addPrismaticJoint(const TVector3 & dir, TScalar jointOffset, TScalar jointFriction, TScalar gearRatio, TScalar rotorInertia, const std::string & name) {
    if (!isOpen) {
        return NULL;
    }
    PrismaticJoint * pj = new PrismaticJoint(*lastElement, dir, getNumberOfJoints(), getDOF(), jointOffset, jointFriction, gearRatio, rotorInertia, name);
    if (addElement(pj)) {
        return pj;
    } else {
        return NULL;
    }
}

RevoluteJoint * MbsCompoundWithBuilder::addRevoluteJoint(const TVector3 & axis, const std::string & name) {
    if (!isOpen) {
        return NULL;
    }
    RevoluteJoint * rj = new RevoluteJoint(*lastElement, axis, getNumberOfJoints(), getDOF(), 0, 0, 0, 0, name);
    if (addElement(rj)) {
        return rj;
    } else {
        return NULL;
    }
}

RevoluteJoint * MbsCompoundWithBuilder::addRevoluteJoint(const TVector3 & axis, TScalar jointOffset, const std::string & name) {
    if (!isOpen) {
        return NULL;
    }
    RevoluteJoint * rj = new RevoluteJoint(*lastElement, axis, getNumberOfJoints(), getDOF(), jointOffset, 0, 0, 0, name);
    if (addElement(rj)) {
        return rj;
    } else {
        return NULL;
    }
}

RevoluteJoint * MbsCompoundWithBuilder::addRevoluteJoint(const TVector3 & axis, TScalar jointOffset, TScalar jointFriction, TScalar gearRatio, TScalar rotorInertia, const std::string & name) {
    if (!isOpen) {
        return NULL;
    }
    RevoluteJoint * rj = new RevoluteJoint(*lastElement, axis, getNumberOfJoints(), getDOF(), jointOffset, jointFriction, gearRatio, rotorInertia, name);
    if (addElement(rj)) {
        return rj;
    } else {
        return NULL;
    }
}

RevoluteJointZ * MbsCompoundWithBuilder::addRevoluteJointZ(TScalar jointOffset, const std::string & name) {
    if (!isOpen) {
        return NULL;
    }
    RevoluteJointZ * rj = new RevoluteJointZ(*lastElement, getNumberOfJoints(), getDOF(), jointOffset, 0, 0, 0, name);
    if (addElement(rj)) {
        return rj;
    } else {
        return NULL;
    }
}

RevoluteJointZ * MbsCompoundWithBuilder::addRevoluteJointZ(TScalar jointOffset, TScalar jointFriction, TScalar gearRatio, TScalar rotorInertia, const std::string & name) {
    if (!isOpen) {
        return NULL;
    }
    RevoluteJointZ * rj = new RevoluteJointZ(*lastElement, getNumberOfJoints(), getDOF(), jointOffset, jointFriction, gearRatio, rotorInertia, name);
    if (addElement(rj)) {
        return rj;
    } else {
        return NULL;
    }
}

RigidLink * MbsCompoundWithBuilder::addRigidLink(const TVector3 & relr, const TVector3 & com, TScalar m, const TMatrix3x3 & I, const std::string & name) {
    if (!isOpen) {
        return NULL;
    }
    RigidLink * rl = new RigidLink(*lastElement, relr, com, m, I, name);
    if (addElement(rl)) {
        return rl;
    } else {
        return NULL;
    }
}

RigidLink * MbsCompoundWithBuilder::addRigidLink(const RigidBodyDescription & rigidBody, const std::string & name) {
    if (!isOpen) {
        return NULL;
    }
    RigidLink * rl = new RigidLink(*lastElement, rigidBody, name);
    if (addElement(rl)) {
        return rl;
    } else {
        return NULL;
    }
}

Drive * MbsCompoundWithBuilder::addDrive(const DriveGenerator & g, const std::string & name) {
    if (!undrivenJoint) {
        return NULL;
    }
    Joint1DOF * j = dynamic_cast< Joint1DOF * >(lastElement);
    if (!j) {
        return NULL;
    }
    Drive * d = g.generateDrive(*j, name);
    if (!d) {
        return NULL;
    }
    MbsCompound::addDrive(d);
    return d;
}

Drive * MbsCompoundWithBuilder::addDrive(const DriveGenerator & g, Joint1DOF & joint, const std::string & name) {
    auto search = jointDriveMap.find(&joint);
    if (search != jointDriveMap.end()) {
        /// Joint already driven
        return nullptr;
    }

    Drive * d = g.generateDrive(joint, name);
    if (!d) {
        return nullptr;
    }
    MbsCompound::addDrive(d);
    jointDriveMap[&joint] = d;
    return d;
}

MbsObject * MbsCompoundWithBuilder::addElement(MbsObject * e) {
    if (e) {
        MbsCompound::addElement(e);
        lastElement = e;
        Joint1DOF * j = dynamic_cast< Joint1DOF * >(e);
        if (j) {
            undrivenJoint = true;
        }
        return e;
    } else {
        return NULL;
    }
}

Spring3D * MbsCompoundWithBuilder::addSpring(const SpringModel & model, const std::string & name) {
    Spring3D * spring = new Spring3D(model, name);
    springs.push_back(spring);
    MbsCompound::addForceGenerator(spring);
    return spring;
}

Spring3D * MbsCompoundWithBuilder::addSpring(const SpringModel & model, std::vector< Endpoint * > points, const std::string & name) {
    Spring3D * spring = new Spring3D(model, points, name);
    springs.push_back(spring);
    addForceGenerator(*spring);
    return spring;
}

Spring1D * MbsCompoundWithBuilder::addSpring(const SpringModel & model, Joint1DOF & j1, Joint1DOF & j2, const std::string & name) {
    Spring1D * spring = new Spring1D(model, j1, j2, name);
    springs.push_back(spring);
    addForceGenerator(*spring);
    return spring;
}

Spring1DSingleEnded * MbsCompoundWithBuilder::addSpring(const SpringModel & model, Joint1DOF & j1, const std::string & name) {
    Spring1DSingleEnded * spring = new Spring1DSingleEnded(model, j1, name);
    springs.push_back(spring);
    addForceGenerator(*spring);
    return spring;
}

Muscle * MbsCompoundWithBuilder::addMuscle(const MuscleModel & model, const std::string & name) {
    Muscle * muscle = new Muscle(model, name);
    muscles.push_back(muscle);
    MbsCompound::addForceGenerator(muscle);
    return muscle;
}

void MbsCompoundWithBuilder::setMuscleControl(TVectorX c) {
    for (size_t i = 0; i < muscles.size(); i++) {
        muscles[i]->setControlValue(c(i));
    }
}

TVectorX MbsCompoundWithBuilder::getMuscleControl(void) const {
    TVectorX c(muscles.size());
    for (size_t i = 0; i < muscles.size(); i++) {
        c(i) = muscles[i]->getControlValue();
    }
    return c;
}

void MbsCompoundWithBuilder::addForceGenerator(ForceGenerator & fg) {
    forceGenerators.push_back(&fg);
    MbsCompound::addForceGenerator(&fg);
}

std::vector< MbsObject * > MbsCompoundWithBuilder::addDHJointLink(JointType jointType, TScalar dh_theta, TScalar dh_d, TScalar dh_a, TScalar dh_alpha, const TVector3 & com, TScalar m, const TMatrix3x3 & I, const std::string & jointName, const std::string & linkName) {
    std::vector< MbsObject * > objects;

    mbslib::TMatrix3x3 rotZ;
    rotZ = Eigen::AngleAxis< mbslib::TScalar >(dh_theta, mbslib::TVector3::UnitZ());
    objects << addFixedRotation(rotZ);

    switch (jointType) {
    case JointType::Revolute:
        objects << addRevoluteJointZ(0.0, jointName);
        break;
    case JointType::Prismatic:
        objects << addPrismaticJoint(mbslib::TVector3::UnitZ(), 0.0, jointName);
        break;
    default:
        break;
    }

    objects << addFixedTranslation(mbslib::TVector3(dh_a, 0.0, dh_d));

    mbslib::TMatrix3x3 rotX;
    rotX = Eigen::AngleAxis< mbslib::TScalar >(dh_alpha, mbslib::TVector3::UnitX());
    objects << addFixedRotation(rotX);
    objects << addFork();
    objects << addFixedTranslation(com);
    objects << addEndpoint(m, I, linkName); // Link mass

    return objects;
}

std::vector< MbsObject * > MbsCompoundWithBuilder::addDHJointLink2(JointType jointType, TScalar dh_theta, TScalar dh_d, TScalar dh_a, TScalar dh_alpha, const TVector3 & com, TScalar m, const TMatrix3x3 & I, const std::string & jointName, const std::string & linkName) {
    std::vector< MbsObject * > objects;

    mbslib::TMatrix3x3 rotZ;
    rotZ = Eigen::AngleAxis< mbslib::TScalar >(dh_theta, mbslib::TVector3::UnitZ());
    objects << addFixedRotation(rotZ);

    switch (jointType) {
    case JointType::Revolute:
        objects << addRevoluteJointZ(0.0, jointName);
        break;
    case JointType::Prismatic:
        objects << addPrismaticJoint(mbslib::TVector3::UnitZ(), 0.0, jointName);
        break;
    default:
        break;
    }

    objects << addFixedTranslation(mbslib::TVector3(dh_a, 0.0, dh_d));

    mbslib::TMatrix3x3 rotX;
    rotX = Eigen::AngleAxis< mbslib::TScalar >(dh_alpha, mbslib::TVector3::UnitX());
    objects << addFixedRotation(rotX);
    objects << addRigidLink(mbslib::TVector3::Zero(), com, m, I, linkName);

    return objects;
}

size_t MbsCompoundWithBuilder::getNumberOfMuscles(void) const {
    return muscles.size();
}

std::vector< Muscle * > & MbsCompoundWithBuilder::getMuscles(void) {
    return muscles;
}

const std::vector< Muscle * > & MbsCompoundWithBuilder::getMuscles(void) const {
    return muscles;
}
