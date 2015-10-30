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
 * \file src/example/exp_17_jfcontroller.cpp
 * 
 */

#include <mbslib/mbslib.hpp>
#include <mbslib/mbslib.muscle.models.hpp>

#include <iostream>

int main(void) {
    mbslib::MbsCompoundWithBuilder b;

    b.addFixedBase();
    mbslib::Joint1DOF * j1 = b.addPrismaticJoint(mbslib::TVector3(0, 0, 1));
    mbslib::JointForceSetter * jfs = new mbslib::JointForceSetter(*j1);
    b.addForceGenerator(*jfs);
    b.addRigidLink(mbslib::TVector3::Zero(), mbslib::TVector3::Zero(), 1, mbslib::TMatrix3x3::Zero());
    b.addEndpoint();

    b.doABA();
    std::cout << "tau = " << b.getJointForceTorque().transpose() << std::endl
              << "ctl = " << b.getControlValues().transpose() << std::endl
              << "ddq = " << b.getJointAcceleration().transpose() << std::endl
              << std::endl;

    mbslib::TVectorX tau(1);
    mbslib::TVectorX ctl(1);

    tau(0) = 1;
    ctl(0) = 0;
    b.setJointForceTorque(tau);
    b.setControlValues(ctl);
    b.doABA();
    std::cout << "tau = " << b.getJointForceTorque().transpose() << std::endl
              << "ctl = " << b.getControlValues().transpose() << std::endl
              << "ddq = " << b.getJointAcceleration().transpose() << std::endl
              << std::endl;

    tau(0) = 0;
    ctl(0) = 1;
    b.setJointForceTorque(tau);
    b.setControlValues(ctl);
    b.doABA();
    std::cout << "tau = " << b.getJointForceTorque().transpose() << std::endl
              << "ctl = " << b.getControlValues().transpose() << std::endl
              << "ddq = " << b.getJointAcceleration().transpose() << std::endl
              << std::endl;

    tau(0) = 1;
    ctl(0) = 1;
    b.setJointForceTorque(tau);
    b.setControlValues(ctl);
    b.doABA();
    std::cout << "tau = " << b.getJointForceTorque().transpose() << std::endl
              << "ctl = " << b.getControlValues().transpose() << std::endl
              << "ddq = " << b.getJointAcceleration().transpose() << std::endl
              << std::endl;

    return 0;
}