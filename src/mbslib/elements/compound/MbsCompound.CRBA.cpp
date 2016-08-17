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

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <iostream>
#include <mbslib/elements/MbsCompound.hpp>
#include <mbslib/elements/base/FixedBase.hpp>
#include <mbslib/elements/base/FreeBase.hpp>
#include <mbslib/utility/internalTests.hpp>
#include <mbslib/utility/mathtools.hpp>

using namespace mbslib;

void MbsCompound::doCrba() {
    doCrba(gravitationVector);
}

void MbsCompound::doCrba(const TVector3 & g) {
    std::vector< PointContact > contacts;
    doCrba(g, contacts);
}

void MbsCompound::doCrba(const std::vector< PointContact > & contacts) {
    doCrba(gravitationVector, contacts);
}

void MbsCompound::doCrba(const TVector3 & g, const std::vector< PointContact > & contacts) {
    int fb = freeBase ? 6 : 0;
    int joints = getNumberOfJoints();

    if ((fb + joints) == 0) {
        return;
    }

    TVectorX tau(fb + joints);
    if (joints > 0) {
        tau.block(fb, 0, joints, 1) = getJointForceTorque();
    }
    if (freeBase) {
        tau.block(0, 0, 6, 1).setZero();
    }

    TMatrixX M;

    //if (!freeBase) {
    //TMatrixX   Mold = calculateMassMatrix();
    //} else {
    M = calculateMassMatrix2();
    //}

    //std::cout << Mold << std::endl << std::endl << M << std::endl << std::endl << Mold-M << std::endl << std::endl;;

    //calculate +C+G+extF
    TVectorX ddq(fb + joints);
    ddq = TVectorX::Zero(fb + joints);
    if (joints > 0) {
        setJointAcceleration(ddq.block(fb, 0, joints, 1));
    }

    //we could use the RNE to do this
    //doRne();
    //instead we use an optimized call, as dirkin already has been done in calculate Mass Matrix

    //store initial acceleration of base
    TVector3 dotomegabase = base->getCoordinateFrame().dotomega;
    TVector3 dotvbase = base->getCoordinateFrame().dotv;

    //set acceleration of base to compensate for gravitation
    base->getCoordinateFrame().dotomega = TVector3::Zero();
    base->getCoordinateFrame().dotv = base->getCoordinateFrame().R.transpose() * -g;

    //calculate forces/torques required to compensate for gravitation, coriolis- and external forces
    doVelocity();
    doAcceleration();
    forceGeneratorSet.resetForces();
    forceGeneratorSet.applyForces();
    doRneInward(true);

    TVectorX tauCGExt(fb + joints);
    if (joints > 0) {
        tauCGExt.block(fb, 0, joints, 1) = getJointForceTorque();
    }
    if (freeBase) {
        tauCGExt.block(0, 0, 3, 1) = base->getRneForceTorque().n;
        tauCGExt.block(3, 0, 3, 1) = base->getRneForceTorque().f;
    }

//std::cout << "taucg" << tauCGExt.transpose() << std::endl;
//std::cout << "tau" << tau.transpose() << std::endl << std::endl;
//std::cout << "M" << std::endl << M << std::endl;
//solve equations of dynamics
//M.lu().solve(tau - tauCGExt, & ddq);

#if EIGEN_WORLD_VERSION >= 3
    Eigen::FullPivLU< TMatrixX > lu(M);
    checkNotZero(lu.determinant());
    ddq = lu.solve(tau - tauCGExt);
#else
    Eigen::LU< TMatrixX > lu(M);
    //checkNotZero(lu.determinant());
    lu.solve(tau - tauCGExt, &ddq);
#endif

    //M.ldlt().solve(tau - tauCGExt, & ddq);
    if (joints > 0) {
        setJointAcceleration(ddq.block(fb, 0, joints, 1));
        setJointForceTorque(tau.block(fb, 0, joints, 1));
    }

    //std::cout << "ddq" << ddq.transpose() << std::endl;
    //std::cout << "tau - taucg" << (tau - tauCGExt).transpose() << std::endl << std::endl;
    //std::cout << "M*ddq" << (M * ddq).transpose() << std::endl << std::endl;

    if (freeBase) {
        //set acceleration of base as calculated by CRBA
        base->getCoordinateFrame().dotomega = ddq.block(0, 0, 3, 1);
        base->getCoordinateFrame().dotv = ddq.block(3, 0, 3, 1);
    } else {
        //restore initial acceleration of base
        base->getCoordinateFrame().dotomega = dotomegabase;
        base->getCoordinateFrame().dotv = dotvbase;
    }

    doVelocity();
    doAcceleration();

    // if we have contacts, we have to consider these now,
    // at least, until we know a better way to do so :-)

    if (contacts.size() != 0) {
        size_t contactCount = contacts.size();
        //calculate jacobians for all contact points
        //std::vector<TMatrix6xX> jacobians; jacobians.resize(contacts.size());
        //std::vector<TVectorX> JTdotD; JTdotD.resize(contacts.size());

        TVectorX deltaV;
        deltaV.resize(contactCount);
        TMatrixX JTdotDall;
        JTdotDall.resize(dof, contactCount);
        TMatrixX MinvJTdotDall;
        MinvJTdotDall.resize(dof, contactCount);

        for (size_t i = 0; i < contacts.size(); i++) {
            //jacobians[i] = calculateJacobian(contacts[i].getEndpoint());
            //std::cout << "J " << i << " : "  << std::endl << jacobians[i] << std::endl;
            //JTdotD[i] = jacobians[i].block(3,0,3,dof).transpose() * contacts[i].getNormalWorldCoordinateSystem();
            //std::cout << "JTdotD " << i << " : " << JTdotD[i].transpose() << std::endl;

            JTdotDall.block(0, i, dof, 1) = calculateJacobian(contacts[i].getEndpoint()).block(3, 0, 3, dof).transpose() * contacts[i].getNormalWorldCoordinateSystem();
            deltaV(i) = -contacts[i].getVelocityLocalCoordinateSystem().dot(contacts[i].getNormalLocalCoordinateSystem());
        }

        //std::cout << "M = " << std::endl << M << std::endl << std::endl;

        //calculate inverse of mass-matrix
        TMatrixX Minv = M.lu().inverse();

        //calculate the mapping of impulses/forces to deltadq/deltaddq
        MinvJTdotDall = Minv * JTdotDall;

        //std::cout << "Minv = " << std::endl << Minv << std::endl << std::endl;

        //create matrix to calculate impulses resulting from contact

        //TMatrixX impulseMap; impulseMap.resize(contactCount,contactCount);
        //for (size_t j = 0; j < contactCount; j++) {
        //  for (size_t i = 0; i < contactCount; i++) {
        //    std::cout << j << ":" << JTdotD[j].transpose() << "   " << i << ":" << JTdotD[i].transpose() << std::endl;
        //    impulseMap(j,i) = JTdotD[j].dot(  ( Minv * JTdotD[i] ) );
        //  }
        //}
        //std::cout << "impulseMap = " << std::endl << impulseMap << std::endl;

        TMatrixX impulseMapNew;
        impulseMapNew = JTdotDall.transpose() * MinvJTdotDall;
        //std::cout << "impulseMapNew = " << std::endl << impulseMapNew << std::endl;

        // solve for magnitude of impulse
        //TVectorX lambda;
        //impulseMap.lu().solve(deltaV,&lambda);
        //std::cout << "lambda = " << lambda.transpose() << std::endl;

        TVectorX lambdaNew;

#if EIGEN_WORLD_VERSION >= 3
        lambdaNew = impulseMapNew.lu().solve(deltaV);
#else
        impulseMapNew.lu().solve(deltaV, &lambdaNew);
#endif
        std::cout << "lambdaNew = " << lambdaNew.transpose() << std::endl;

        // distribute impulse into joints
        //TVectorX deltadq; deltadq.resize(dof); deltadq.setZero();
        //for (size_t i = 0; i < contactCount; i++) { deltadq += Minv * JTdotD[i] * lambda(i); }
        //deltadq =  deltadq;
        //std::cout << "deltadq = " << deltadq.transpose() << std::endl;

        TVectorX deltadqNew = MinvJTdotDall * lambdaNew;
        std::cout << "deltadqNew = " << deltadqNew.transpose() << std::endl;

        setJointVelocity(getJointVelocity() + deltadqNew.block(fb, 0, joints, 1));
        if (freeBase) {
            base->getCoordinateFrame().spatialVelocity += deltadqNew.block(0, 0, 6, 1);
        }

        doVelocity();
        doAcceleration();

        //TODO: now we can calculate appropriate contact FORCES to get Acc = 0 in contacts
    }
}
