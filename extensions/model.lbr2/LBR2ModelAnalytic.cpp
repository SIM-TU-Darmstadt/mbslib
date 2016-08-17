/*
 * Copyright (C) 2016
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

#include <Eigen/LU>
#include <Eigen/QR>
#include <model.lbr2/LBR2ModelAnalytic.hpp>
#include <model.lbr2/dlrlbr2.hpp>

using namespace mbslib;
LBR2ModelAnalytic::LBR2ModelAnalytic() {
}
LBR2ModelAnalytic::~LBR2ModelAnalytic() {
}

LBR2ModelAnalytic::JointAccelerations LBR2ModelAnalytic::forwardDynamics(const JointPositions & q, const JointVelocities & qdot, const JointTorques & tau) {
    JointAccelerations qdotdot(7);

    JointTorques torque_g = gravityTorque(q);
    JointTorques torque_c = coriolisTorque(q, qdot);
    Matrix M = massMatrix(q);

    qdotdot = M.fullPivHouseholderQr().solve(tau - torque_g - torque_c);

    return qdotdot;
}

LBR2ModelAnalytic::JointTorques LBR2ModelAnalytic::inverseDynamics(const JointPositions & q, const JointVelocities & qdot, const JointAccelerations & qdotdot) {
    JointTorques tau(7);

    JointTorques torque_g = gravityTorque(q);
    JointTorques torque_c = coriolisTorque(q, qdot);
    Matrix M = massMatrix(q);

    tau = M * qdotdot + torque_c + torque_g;

    return tau;
}

LBR2ModelAnalytic::JointTorques LBR2ModelAnalytic::gravityTorque(const JointPositions & q) {
    return DLRLBR2_G(q);
}

LBR2ModelAnalytic::JointTorques LBR2ModelAnalytic::coriolisTorque(const JointPositions & q, const JointVelocities & qdot) {
    return DLRLBR2_C(q, qdot);
}

LBR2ModelAnalytic::JointTorques LBR2ModelAnalytic::massTorque(const JointPositions & q, const JointAccelerations & qdotdot) {
    return massMatrix(q) * qdotdot;
}

LBR2ModelAnalytic::Matrix LBR2ModelAnalytic::massMatrix(const JointPositions & q) {
    return DLRLBR2_M(q);
}

LBR2ModelAnalytic::Vector3 LBR2ModelAnalytic::forwardKinematics(const JointPositions & q) {
    Vector3 tcp;

    tcp = DLRLBR2_DIRKIN_xyz(q);

    return tcp;
}
