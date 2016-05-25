#define BOOST_TEST_MODULE 1DOF_Pendulum
#include <boost/test/included/unit_test.hpp>

#include <iostream>
#include <stdio.h>
#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>
#include <mbslib/elements/joint/JointForceSetter.hpp>

using namespace mbslib;

/**
 * \brief Calculate direct kinematics
 * 		x = l sin(q) ,		y = -l cos(q)
 *
 * \param pendulumLength length of pendulum
 * \param q joint position
 */
TVector3 calculateDirectKinematics(TScalar pendulumLength, TVectorX q) {
    TVector3 xyCoordinates = TVector3::Zero();

    xyCoordinates(0) = pendulumLength * std::sin(q(0));
    xyCoordinates(1) = -pendulumLength * std::cos(q(0));

    return xyCoordinates;
}

/**
 * \brief Calculate direct dynamics
 * 		ddq = 3(tau - m * gravity * (l/2) * sin(q))
 * 		      ---------------------------
 * 		                m * l^2
 *
 * \param pendulumLength length of pendulum
 * \param q joint position
 * \param tau joint torque
 * \param gravity gravityity
 * \param pendulumMass
 */
TVectorX calculateDirectDynamics(TScalar pendulumLength, TVectorX q, TScalar tau, TScalar gravity, TScalar pendulumMass) {
    TVectorX ddq(1);
    ddq(0) = 3 * (tau - (pendulumMass * gravity * (pendulumLength / 2) * std::sin(q(0)))) / (pendulumMass * pendulumLength * pendulumLength);

    return ddq;
}

/**
 * \brief Calculate inverse dynamics
 * 		tau = (1/3) * m * l^2 * ddq  +  m * gravity * (l/2) * sin(q)
 *
 *
 * \param pendulumLength length of pendulum
 * \param q joint position
 * \param ddq joint force acceleration
 * \param gravity gravity
 * \param pendulumMass
 */
TScalar calculateInverseDynamics(TScalar pendulumLength, TVectorX q, TVectorX ddq, TScalar gravity, TScalar pendulumMass) {
    TScalar tau = (pendulumMass * pendulumLength * pendulumLength * ddq(0)) / 3 + pendulumMass * gravity * (pendulumLength / 2) * std::sin(q(0));
    return tau;
}

MbsCompoundWithBuilder * buildKinematicsModel(TScalar pendulumLength, TVector3 gravity = TVector3::Zero(3)) {
    MbsCompoundWithBuilder * mbs = new MbsCompoundWithBuilder("Pendulum1DOF");
    mbs->addFixedBase();
    mbs->setGravitation(gravity);
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "q");
    mbs->addFixedTranslation(mbslib::TVector3(0, -pendulumLength, 0));
    mbs->addEndpoint("pendulum endpoint");
    return mbs;
}
MbsCompoundWithBuilder * buildDynamicsModel(TScalar pendulumLength, TVector3 gravity = TVector3::Zero(3), TScalar pendulumMass = 0.0, TVector3 pendulumCOM = TVector3::Zero(3), TMatrixX pendulumInertia = TMatrix3x3::Zero()) {
    MbsCompoundWithBuilder * mbs = new MbsCompoundWithBuilder("Pendulum1DOF");
    mbs->addFixedBase();
    mbs->setGravitation(gravity);
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "q");
    mbs->addRigidLink(TVector3::UnitY() * -pendulumLength, pendulumCOM, pendulumMass, pendulumInertia, "pendulum");
    mbs->addEndpoint("pendulum endpoint");
    return mbs;
}

BOOST_AUTO_TEST_CASE(DirectKinematics) {
    TScalar pendulumLength = 10; //pendulumLength of pendulum arm
    TVectorX q(1);

    TVector3 mbslibResult;
    TVector3 analyticalResult;
    Endpoint * endPoint;
    MbsCompoundWithBuilder * mbs;

    for (TScalar q_counter = 0.0; q_counter < 500.0; q_counter += 10.0) {
        q << q_counter / 180.0 * M_PI;
        for (TScalar l_counter = 0.0; l_counter < 50.0; l_counter += 1.0) {
            pendulumLength = l_counter / 10.0;
            mbs = buildKinematicsModel(pendulumLength);
            endPoint = mbs->getEndpointByName("pendulum endpoint");

            mbs->setJointPosition(q);

            mbs->doDirkin();

            mbslibResult = endPoint->getCoordinateFrame().r;

            analyticalResult = calculateDirectKinematics(pendulumLength, q);

            //std::cout << "mbslib result: " << std::endl << mbslibResult << std::endl;
            //std::cout << "manual calculation result: " << std::endl << manualResult << std::endl;

            BOOST_CHECK((mbslibResult - analyticalResult).isZero());

            delete mbs;
        }
    }
}

BOOST_AUTO_TEST_CASE(DirectDynamics) {
    TScalar pendulumLength = 0.0;
    TVectorX q(1);
    TScalar tau = 0.0;

    TVectorX mbslibResult(1);
    TVectorX analyticalResult(1);

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    for (TScalar tau = -10.0; tau <= 10.0; tau += 5) {
        for (TScalar gravity = -10.0; gravity <= 10.0; gravity += 5) {
            for (TScalar pendulumMass = 5.0; pendulumMass <= 30.0; pendulumMass += 5.0) {
                for (TScalar q_counter = 0.0; q_counter <= 500.0; q_counter += 25.0) {
                    q << q_counter / 180.0 * M_PI;
                    for (TScalar l_counter = 1.0; l_counter <= 51.0; l_counter += 2.5) {
                        pendulumLength = l_counter / 10.0;
                        MbsCompoundWithBuilder * mbs;
                        TMatrix3x3 pendulumInertia = TMatrix3x3::Zero(); // inertia of arm
                        pendulumInertia(0, 0) = pendulumInertia(2, 2) = pendulumMass * pendulumLength * pendulumLength / 12;

                        mbs = buildDynamicsModel(pendulumLength, TVector3(0.0, -gravity, 0.0), pendulumMass, TVector3::UnitY() * pendulumLength / 2, pendulumInertia);
                        Joint1DOF * joint1 = mbs->getJointByName("q");

                        mbs->setJointPosition(q);
                        joint1->setJointForceTorque(tau);

                        JointForceSetter * jfs1 = new JointForceSetter(*joint1);
                        mbs->addForceGenerator(*jfs1);

                        mbs->doABA();

                        mbslibResult = mbs->getJointAcceleration();
                        analyticalResult = calculateDirectDynamics(pendulumLength, q, tau, gravity, pendulumMass);
                        BOOST_CHECK((mbslibResult - analyticalResult).isZero());

                        mbs->doCrba();

                        mbslibResult = mbs->getJointAcceleration();
                        analyticalResult = calculateDirectDynamics(pendulumLength, q, tau, gravity, pendulumMass);
                        BOOST_CHECK((mbslibResult - analyticalResult).isZero());
                        delete mbs;
                        delete jfs1;
                    }
                }
            }
        }
    }

    //Eigen::IOFormat LongFormat(20);
    //std::cout << "mbslib result: " << std::endl << mbslibResult.format(LongFormat) << std::endl;
    //std::cout << analyticalResult.format(LongFormat) << std::endl;
}

BOOST_AUTO_TEST_CASE(InverseDynamics) {
    TScalar pendulumLength = 1; //pendulumLength of pendulum arm
    TVectorX q(1);
    q << (3.14 / 2);
    TVectorX ddq(1);

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    for (ddq << -10.0; ddq(0) <= 10.0; ddq(0) += 5) {
        for (TScalar gravity = -10.0; gravity <= 10.0; gravity += 5) {
            for (TScalar pendulumMass = 5.0; pendulumMass <= 30.0; pendulumMass += 5.0) {
                for (TScalar q_counter = 0.0; q_counter <= 500.0; q_counter += 25.0) {
                    q << q_counter / 180.0 * M_PI;
                    for (TScalar l_counter = 1.0; l_counter <= 51.0; l_counter += 2.5) {
                        pendulumLength = l_counter / 10.0;

                        MbsCompoundWithBuilder * mbs;
                        TMatrix3x3 pendulumInertia = TMatrix3x3::Zero();           // inertia of arm
                        TVector3 pendulumCOM = TVector3::UnitY() * pendulumLength / 2; // centor of pendulumMass of arm
                        pendulumInertia(0, 0) = pendulumInertia(2, 2) = pendulumMass * pendulumLength * pendulumLength / 12;

                        mbs = buildDynamicsModel(pendulumLength, TVector3(0.0, -gravity, 0.0), pendulumMass, pendulumCOM, pendulumInertia);
                        Joint1DOF * joint1 = mbs->getJointByName("q");

                        mbs->setJointPosition(q);
                        mbs->setJointAcceleration(ddq);

                        mbs->doRne();

                        TScalar mbslibResult = joint1->getJointForceTorque();
                        TScalar analyticalResult = calculateInverseDynamics(pendulumLength, q, ddq, gravity, pendulumMass);

                        //std::cout << "mbslib result: " << std::endl << mbslibResult << std::endl;
                        //std::cout << "inverse result: " << analyticalResult << std::endl;

                        BOOST_CHECK(std::fabs(mbslibResult - analyticalResult) < 1e-12);
                        //BOOST_CHECK((mbslibResult-analyticalResult).isZero());
                    }
                }
            }
        }
    }
}
