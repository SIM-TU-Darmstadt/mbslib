// ------------------------------------------------------
// Technische Universität Darmstadt
// Department of Computer Science
// Simulation, Systems Optimization and Robotics Group
// Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
// Licensed under BSD 3-Clause License
// ------------------------------------------------------

// Defines
#define USE_ADOLC

// Includes
#include <iostream>
#include <string>
#include <mbslib/mbslib.hpp>
#include <mbslib/elements/drive/ActiveSpringDamperDrive.hpp>

extern "C" {

#include "biorob.h"

// Global variables
struct parameters {

    parameters() {

        g = 9.81;
        k1 = 45.00;
        d1 = 0.22;
        l1 = 0.28;
        m1 = 2.58;
        com1 = mbslib::TVector3(-3.38e-3, -2.02e-3, -133.83e-3);
        it1 = mbslib::makeInertiaTensor(2861.88e-5, 2061.95e-5, 1419.69e-5, 0, 0, 0);
        k2 = 40.00;
        d2 = 0.18;
        l2 = 0.31;
        m2 = 1.42;
        com2 = mbslib::TVector3(-8.56e-3, 3.42e-3, -301.76e-3);
        it2 = mbslib::makeInertiaTensor(2020.67e-5, 1937.18e-5, 184.56e-5, 0, 0, 0);
        k3 = 25.00;
        d3 = 0.12;
        l3 = 0.31;
        m3 = 0.66;
        com3 = mbslib::TVector3(-0.02e-3, 16.837e-3, -199.94e-3);
        it3 = mbslib::makeInertiaTensor(841.27e-5, 812.42e-5, 51.88e-5, 0, 0, 0);
        k4 = 15.00;
        d4 = 0.14;
        l4 = 0.17;
        m4 = 0.18;
        com4 = mbslib::TVector3(0.06e-3, -0.6e-3, -52.51e-3);
        it4 = mbslib::makeInertiaTensor(14.04e-5, 13.46e-5, 5.67e-5, 0, 0, 0);
    }

    mbslib::TScalar g;
    mbslib::TScalar k1;
    mbslib::TScalar d1;
    mbslib::TScalar l1;
    mbslib::TScalar m1;
    mbslib::TVector3 com1;
    mbslib::TMatrix3x3 it1;
    mbslib::TScalar k2;
    mbslib::TScalar d2;
    mbslib::TScalar l2;
    mbslib::TScalar m2;
    mbslib::TVector3 com2;
    mbslib::TMatrix3x3 it2;
    mbslib::TScalar k3;
    mbslib::TScalar d3;
    mbslib::TScalar l3;
    mbslib::TScalar m3;
    mbslib::TVector3 com3;
    mbslib::TMatrix3x3 it3;
    mbslib::TScalar k4;
    mbslib::TScalar d4;
    mbslib::TScalar l4;
    mbslib::TScalar m4;
    mbslib::TVector3 com4;
    mbslib::TMatrix3x3 it4;
};
parameters * p = nullptr;
mbslib::MbsCompoundWithBuilder * mbs = nullptr;
mbslib::DeriveOMat * dom = nullptr;
mbslib::Joint1DOF * jo1 = nullptr;
mbslib::Joint1DOF * jo2 = nullptr;
mbslib::Joint1DOF * jo3 = nullptr;
mbslib::Joint1DOF * jo4 = nullptr;
mbslib::ActiveSpringDamperDrive * sd1 = nullptr;
mbslib::ActiveSpringDamperDrive * sd2 = nullptr;
mbslib::ActiveSpringDamperDrive * sd3 = nullptr;
mbslib::ActiveSpringDamperDrive * sd4 = nullptr;

// Global functions
void createModel(void) {

    // Initialize variables
    p = new parameters();
    mbs = new mbslib::MbsCompoundWithBuilder("BioRob");
    dom = new mbslib::DeriveOMat(*mbs);

    // Add gravity
    mbs->setGravitation(mbslib::TVector3(0, 0, -p->g));

    // Add fixed base, joints and links
    mbs->addFixedBase("b");
    jo1 = mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "jo1");
    mbs->addRigidLink(mbslib::TVector3(0, 0, p->l1), p->com1, p->m1, p->it1, "rl1");
    jo2 = mbs->addRevoluteJoint(mbslib::TVector3(0, 1, 0), "jo2");
    mbs->addRigidLink(mbslib::TVector3(0, 0, p->l2), p->com2, p->m2, p->it2, "rl2");
    jo3 = mbs->addRevoluteJoint(mbslib::TVector3(0, 1, 0), "jo3");
    mbs->addRigidLink(mbslib::TVector3(0, 0, p->l3), p->com3, p->m3, p->it3, "rl3");
    jo4 = mbs->addRevoluteJoint(mbslib::TVector3(0, 1, 0), "jo4");
    mbs->addRigidLink(mbslib::TVector3(0, 0, p->l4), p->com4, p->m4, p->it4, "rl4");
    mbs->addEndpoint("e");

    jo1->setJointPosition(M_PI / 2);

    // Add springs and dampers
    sd1 = new mbslib::ActiveSpringDamperDrive(*jo1, p->k1, p->d1, "sd1");
    sd2 = new mbslib::ActiveSpringDamperDrive(*jo2, p->k2, p->d2, "sd2");
    sd3 = new mbslib::ActiveSpringDamperDrive(*jo3, p->k3, p->d3, "sd3");
    sd4 = new mbslib::ActiveSpringDamperDrive(*jo4, p->k4, p->d4, "sd4");
    sd1->setDesiredPosition(0);
    sd2->setDesiredPosition(0);
    sd3->setDesiredPosition(0);
    sd4->setDesiredPosition(0);
    sd1->setDesiredVelocity(0);
    sd2->setDesiredVelocity(0);
    sd3->setDesiredVelocity(0);
    sd4->setDesiredVelocity(0);
    ((mbslib::MbsCompound *)mbs)->addDrive(sd1);
    ((mbslib::MbsCompound *)mbs)->addDrive(sd2);
    ((mbslib::MbsCompound *)mbs)->addDrive(sd3);
    ((mbslib::MbsCompound *)mbs)->addDrive(sd4);

    // Add independent variables
    dom->addIndependent("sd1", "springconstant");
    dom->addIndependent("sd2", "springconstant");
    dom->addIndependent("sd3", "springconstant");
    dom->addIndependent("sd4", "springconstant");
    dom->addIndependent("sd1", "springdamping");
    dom->addIndependent("sd2", "springdamping");
    dom->addIndependent("sd3", "springdamping");
    dom->addIndependent("sd4", "springdamping");

    // Add dependent variables
    dom->addDependent("jo1", "ddq");
    dom->addDependent("jo2", "ddq");
    dom->addDependent("jo3", "ddq");
    dom->addDependent("jo4", "ddq");
}

void setJointStates(double * jointPositions, double * jointVelocities) {

    if ((mbs != nullptr) && (jointPositions != nullptr) && (jointVelocities != nullptr)) {

        // Set joint positions
        jo1->setJointPosition(jointPositions[0]);
        jo2->setJointPosition(jointPositions[1]);
        jo3->setJointPosition(jointPositions[2]);
        jo4->setJointPosition(jointPositions[3]);

        // Set joint velocities
        jo1->setJointVelocity(jointVelocities[0]);
        jo2->setJointVelocity(jointVelocities[1]);
        jo3->setJointVelocity(jointVelocities[2]);
        jo4->setJointVelocity(jointVelocities[3]);

    } else {

        // Print error
        std::cout << "ERROR: There were invalid pointers when calling setJointStates function!" << std::endl;
    }
}

void applyForwardDynamicsReference(double * motorPositions) {

    if ((mbs != nullptr) && (motorPositions != nullptr)) {

        double stiffnessConstants[4] = {p->k1.getValue(), p->k2.getValue(), p->k3.getValue(), p->k4.getValue()};
        double dampingConstants[4] = {p->d1.getValue(), p->d2.getValue(), p->d3.getValue(), p->d4.getValue()};
        applyForwardDynamics(motorPositions, stiffnessConstants, dampingConstants);

    } else {

        // Print error
        std::cout << "ERROR: There were invalid pointers when calling applyForwardDynamicsReference function!" << std::endl;
    }
}

void applyForwardDynamics(double * motorPositions, double * stiffnessConstants, double * dampingConstants) {

    if ((mbs != nullptr) && (motorPositions != nullptr) && (stiffnessConstants != nullptr) && (dampingConstants != nullptr)) {

        // Set motor positions
        sd1->setDesiredPosition(motorPositions[0]);
        sd2->setDesiredPosition(motorPositions[1]);
        sd3->setDesiredPosition(motorPositions[2]);
        sd4->setDesiredPosition(motorPositions[3]);

        // Set stiffness constants
        sd1->setParameter(2, stiffnessConstants[0]);
        sd2->setParameter(2, stiffnessConstants[1]);
        sd3->setParameter(2, stiffnessConstants[2]);
        sd4->setParameter(2, stiffnessConstants[3]);

        // Set damping constants
        sd1->setParameter(3, dampingConstants[0]);
        sd2->setParameter(3, dampingConstants[1]);
        sd3->setParameter(3, dampingConstants[2]);
        sd4->setParameter(3, dampingConstants[3]);

        // Start tape for derivatives
        dom->startTape();

        // Apply forward dynamics
        mbs->doForwardDrives();
        mbs->doCrba();

        // Stop tape for derivatives
        dom->endTape();

    } else {

        // Print error
        std::cout << "ERROR: There were invalid pointers when calling applyForwardDynamics function!" << std::endl;
    }
}

void getForwardDynamicsResults(double * jointAccelerations) {

    if ((mbs != nullptr) && (jointAccelerations != nullptr)) {

        jointAccelerations[0] = jo1->getJointAcceleration().getValue();
        jointAccelerations[1] = jo2->getJointAcceleration().getValue();
        jointAccelerations[2] = jo3->getJointAcceleration().getValue();
        jointAccelerations[3] = jo4->getJointAcceleration().getValue();

    } else {

        // Print error
        std::cout << "ERROR: There were invalid pointers when calling getForwardDynamicsResults function!" << std::endl;
    }
}

void getForwardDynamicsJacobian(double * jacobianVector) {

    mbslib::DomMatrix m = dom->calculateJacobian();
    mbslib::DomVector v1 = mbslib::convert(m);
    Eigen::Map< Eigen::VectorXd > v2(jacobianVector, v1.size());
    v2 = v1;
}
}
