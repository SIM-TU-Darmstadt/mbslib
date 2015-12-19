// ------------------------------------------------------
// Technische Universität Darmstadt
// Department of Computer Science
// Simulation, Systems Optimization and Robotics Group
// Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
// Licensed under BSD 3-Clause License
// ------------------------------------------------------

#ifndef BIOROB_H
#define BIOROB_H

// Global prototypes
void createModel(void);
void setJointStates(double * jointPositions, double * jointVelocities);
void applyForwardDynamicsReference(double * motorPositions);
void applyForwardDynamics(double * motorPositions, double * stiffnessConstants, double * dampingConstants);
void getForwardDynamicsResults(double * jointAccelerations);
void getForwardDynamicsJacobian(double * jacobianVector);

#endif
