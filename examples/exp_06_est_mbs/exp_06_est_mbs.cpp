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
 * \file src/example/exp_06_est_mbs.cpp
 * 
 */
#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>
#include <adolc/drivers/drivers.h>

using namespace mbslib;

mbslib::MbsCompoundWithBuilder * createRobot(TScalar l1, TScalar m1, TScalar l2, TScalar m2) {
    mbslib::MbsCompoundWithBuilder * b = new mbslib::MbsCompoundWithBuilder();
    b->addFixedBase("base");
    b->addRevoluteJoint(Eigen::Vector3d::UnitZ().cast< TScalar >(), 0, "j1");
    b->addRigidLink(TVector3(l1, 0, 0), TVector3(0, 0, 0), m1, Eigen::Matrix3d::Zero().cast< TScalar >(), "l1");
    b->addRevoluteJoint(Eigen::Vector3d::UnitZ().cast< TScalar >(), 0, "j2");
    b->addRigidLink(TVector3(l2, 0, 0), TVector3(0, 0, 0), m2, Eigen::Matrix3d::Zero().cast< TScalar >(), "l2");
    b->addEndpoint("tcp");

    b->setGravitation(TVector3(0, -1, 0));
    return b;
}

int deriveForJointVariables(void) {
    // at first we get only derivatives fromt he joint values
    // so we do not need to write the tape while creating the robot.
    mbslib::MbsCompound * mbs = createRobot(1, 1, 1, 1);

    // variables to correspond to our independents
    double q1 = 0;
    double q2 = 0;

    // and variables to correspond to the dependents
    double x, y, z;

    trace_on(0);

    // define the independents
    mbs->getJoints().at(0)->getJointState().q <<= q1;
    mbs->getJoints().at(1)->getJointState().q <<= q2;

    // do the calculation
    mbs->doDirkin();

    // get the dependents
    TVector3 tcp = mbs->getEnd().getCoordinateFrame().r;
    tcp.x() >>= x;
    tcp.y() >>= y;
    tcp.z() >>= z;

    trace_off();

    // Variables to play with
    // joint positions
    double q[2];
    // tcp position
    double r[3];
    // jacobian: 3 rows of two elements each
    double J0[2];
    double J1[2];
    double J2[2];
    double * J[3] = {J0, J1, J2};

    // Calculate dirkin and jacobian
    q[0] = 0;
    q[1] = 0;

    ::function(0, 3, 2, q, r);
    ::jacobian(0, 3, 2, q, J);

    delete mbs;

    return 0;
}

int deriveForLinkParameters(void) {
    // variables to correspond to our independents
    double q1_ = 0;
    double q2_ = 0;
    double l1_ = 1;
    double l2_ = 1;

    // and variables to correspond to the dependents
    double x_, y_, z_;

    trace_on(0);

    // define the independents
    // note: an adouble becomes an independent for a given tape
    // when we <<= a value to it. The value need not be from a
    // variable but also my come from a constant.
    // by <<=ing something to an adouble we set it's value for
    // the upcomming evaluation of the tape (at least MF thinks this)
    TScalar q1;
    q1 <<= q1_;
    TScalar q2;
    q2 <<= q2_;
    TScalar l1;
    l1 <<= l1_;
    TScalar l2;
    l2 <<= l2_;

    // create robot depending on two of the independents.
    mbslib::MbsCompound * mbs = createRobot(l1, 1, l2, 1);

    // set joint variables with two further independents.
    mbs->getJoints().at(0)->getJointState().q = q1;
    mbs->getJoints().at(1)->getJointState().q = q2;

    // do the calculation
    mbs->doDirkin();

    // get the dependents
    TVector3 tcp = mbs->getEnd().getCoordinateFrame().r;
    tcp.x() >>= x_;
    tcp.y() >>= y_;
    tcp.z() >>= z_;

    trace_off();

    // Variables to play with
    // joint positions and length in order of defition of the independents
    // (order is q1 q2 l1 l2!)
    double indep[4];
    // tcp position (the only dependents)
    double dep[3];
    // jacobian: 3 rows of four elements each
    double J0[4];
    double J1[4];
    double J2[4];
    double * J[3] = {J0, J1, J2};

    // Calculate dirkin and jacobian
    indep[0] = 0;
    indep[1] = 0;
    indep[2] = 1;
    indep[3] = 1;
    ::function(0, 3, 4, indep, dep);
    ::jacobian(0, 3, 4, indep, J);

    delete mbs;

    // as we can calculate the hessian only for one dependent
    // we have to set up a new tape

    trace_on(1);
    q1 <<= 0; //q1_;
    q2 <<= 0; //q2_;
    l1 <<= 1; //l1_;
    l2 <<= 1; //l2_;

    mbs = createRobot(l1, 1, l2, 1);

    mbs->getJoints().at(0)->getJointState().q = q1;
    mbs->getJoints().at(1)->getJointState().q = q2;

    mbs->doDirkin();

    TScalar y = mbs->getEnd().getCoordinateFrame().r.y();
    y >>= y_;

    trace_off();

    // hessian only for one component of tcp 4 rows of 4 elements
    double H0[4];
    double H1[4];
    double H2[4];
    double H3[4];

    double * H[4] = {H0, H1, H2, H3};

    hessian(1, 4, indep, H);

    delete mbs;

    return 0;
}

int deriveInvDynForParameters(void) {
    trace_on(0);

    // independents
    TVectorX q(2);
    TScalar l1;
    TScalar l2;

    q(0) <<= 0;
    q(1) <<= 0;
    l1 <<= 1;
    l2 <<= 1;

    // run algorithm
    mbslib::MbsCompound * mbs = createRobot(l1, 1, l2, 1);

    mbs->setJointPosition(q);

    TVectorX tauG = mbs->calculateGravitationVectorInJoints();

    mbs->doCrba();

    // dependents
    double tau1;
    tauG(0) >>= tau1;
    double tau2;
    tauG(1) >>= tau2;

    trace_off();

    delete mbs;

    // again we fool around with the model

    double indep[4] = {0, 0, 1, 2};
    double dep[2];

    ::function(0, 2, 4, indep, dep);

    return 0;
}

int deriveCRBAForParameters() {
    trace_on(0);

    // independents
    TScalar m1;
    TScalar m2;

    m1 <<= 1;
    m2 <<= 1;

    // run some calculations
    mbslib::MbsCompound * mbs = createRobot(1, m1, 1, m2);
    mbs->doDirkin();

    TVectorX tau(2);
    tau(0) = 3;
    tau(1) = 1; // lazy MF: tau is not independent, so it stays the same for all later evaluations
    mbs->setJointForceTorque(tau);
    mbs->doCrba();

    TVectorX ddq = mbs->getJointAcceleration();

    // dependents
    double ddq1;
    double ddq2;
    ddq(0) >>= ddq1;
    ddq(1) >>= ddq2;

    delete mbs;
    trace_off();

    // play with the values ...
    double indep[2] = {1.1, 1};
    double dep[2];
    double J1[2];
    double J2[2];
    double * J[2] = {J1, J2};

    ::function(0, 2, 2, indep, dep);
    jacobian(0, 2, 2, indep, J);

    return 0;
}

int main(void) {
    return deriveCRBAForParameters();
}
