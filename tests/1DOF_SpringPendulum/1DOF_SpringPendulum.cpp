#define BOOST_TEST_MODULE 1DOF_SpringPendulum
#include <boost/test/included/unit_test.hpp>

#include <fstream>
#include <iostream>
#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>
#include <stdio.h>
#include <mbslib/elements/spring/model/LinearSpringModel.hpp>
#include <mbslib/utility/internalTests.hpp>

//#define ERROR_PRINTOUT

//#define BOOST_AUTO_TEST_CASE_MT(TESTCASE) \
//

using namespace mbslib;

/**
 * \brief Calculate direct kinematics
 * 		x = 0 ,		y = - q
 *
 * \param q joint position
 */
TVector3 calculateDirectKinematics(TVectorX q) {
    TVector3 xyCoordinates = TVector3::Zero();

    xyCoordinates(0) = 0;
    xyCoordinates(1) = -q(0);

    return xyCoordinates;
}

/**
 * \brief Calculate direct dynamics
 * 		ddq = m*g - d*dq - k*q + tau
 * 		      ---------------------
 * 		                m
 *
 * \param q joint position
 * \param dq joint velocity
 * \param tau joint torque
 * \param g gravity
 * \param m mass
 * \param d
 * \param k
 */
TVectorX calculateDirectDynamics(TVectorX q, TVectorX dq, TScalar tau, TScalar g, TScalar m, TScalar k, TScalar d) {
    TVectorX ddq(1);

    ddq(0) = g + (tau - d * dq(0) - k * q(0)) / m;

    return ddq;
}

/**
 * \brief Calculate inverse dynamics
 * 		tau = m*ddq + d*dq + k*q - m*g
 *
 *
 * \param q joint position
 * \param dq joint force velocity
 * \param ddq joint force acceleration
 * \param g gravity
 * \param m mass
 * \param d
 * \param k
 */
TScalar calculateInverseDynamics(TVectorX q, TVectorX dq, TVectorX ddq, TScalar g, TScalar m, TScalar k, TScalar d) {
    TScalar tau = m * ddq(0) + d * dq(0) + k * q(0) - m * g;
    return tau;
}

MbsCompoundWithBuilder * buildModel(TScalar pendulumMass, TScalar springConstant, TScalar damperConstant, TVector3 gravity = TVector3::Zero(3)) {
    MbsCompoundWithBuilder * mbs = new MbsCompoundWithBuilder("Pendulum1DOF");
    mbs->addFixedBase();
    mbs->setGravitation(gravity);
    mbs->addFork();
    mbs->addPrismaticJoint(-TVector3::UnitY(), "q");
    mbs->addEndpoint(pendulumMass, TMatrix3x3::Zero(), "pendulum endpoint");
    mbs->addEndpointMassless("pendulum startpoint");
    LinearSpringModel springModel(springConstant, damperConstant);
    mbs->addSpring(springModel, mbs->getEndpoints());
   return mbs;
}

BOOST_AUTO_TEST_CASE(DirectKinematics) {
    TVectorX q(1);

    TVector3 mbslibResult;
    TVector3 analyticalResult;
    Endpoint * endPoint;
    MbsCompoundWithBuilder * mbs;

    const MMSTs _q_counter          {  0.0,  500.0,  1.0};

    mbs = buildModel(1.0,1.0,1.0);
    endPoint = mbs->getEndpointByName("pendulum endpoint");
    for (q(0) = _q_counter.min; q(0) <= _q_counter.max; q(0) += _q_counter.step) {
        mbs->setJointPosition(q);
        mbs->doDirkin();

        mbslibResult = endPoint->getCoordinateFrame().r;
        analyticalResult = calculateDirectKinematics(q);

        BOOST_CHECK((mbslibResult - analyticalResult).isZero());
    }
    delete mbs;
}
//#define ERROR_PRINTOUT
BOOST_AUTO_TEST_CASE(DirectDynamics) {
    TVectorX mbslibResult(1);
    TVectorX analyticalResult(1);
    TVectorX q(1), dq(1);
    TScalar tau = 0.0;

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    // min, max, step
    const MMSTs _tau                {-10.0,   10.0,   5.0};
    const MMSTs _gravity            {-10.0,   10.0,   5.0};
    const MMSTs _pendulumMass       {  5.0,   30.0,   5.0};
    const MMSTs _q_counter          {  0.0,  500.0,  25.0};
    const MMSTs _dq_counter         {  0.0,   50.0,  10.0};
    const MMSTs _springConstant     {  1.0,  151.0,   5.0};
    const MMSTs _damperConstant     {  0.0,    5.0,   0.5};

    for (TScalar gravity = _gravity.min; gravity <= _gravity.max; gravity += _gravity.step) {
    for (TScalar pendulumMass = _pendulumMass.min; pendulumMass <= _pendulumMass.max; pendulumMass += _pendulumMass.step) {
    for (TScalar springConstant = _springConstant.min; springConstant <= _springConstant.max; springConstant += _springConstant.step) {
    for (TScalar damperConstant = _damperConstant.min; damperConstant <= _damperConstant.max; damperConstant += _damperConstant.step) {
        MbsCompoundWithBuilder * mbs;

        mbs = buildModel(pendulumMass, springConstant, damperConstant, TVector3(0.0, -gravity, 0.0));
        Joint1DOF * joint1 = mbs->getJointByName("q");

        for (TScalar tau = _tau.min; tau <= _tau.max; tau += _tau.step) {
        for (q(0) = _q_counter.min; q(0) <= _q_counter.max; q(0) += _q_counter.step) {
        for (dq(0) = _dq_counter.min; dq(0) <= _dq_counter.max; dq(0) += _dq_counter.step) {
#ifdef ERROR_PRINTOUT
            auto printData = [&]() {
                std::cout << "=================================" << std::endl;
                std::cout << "k " << springConstant << " d " << damperConstant << std::endl;
                std::cout << "q " << q << " dq " << dq << std::endl;
                std::cout << "m " << pendulumMass << " g " << gravity << " tau " << tau << std::endl;
            };
            auto printResults = [&]() {
                std::cout << "mbslib result: " << std::endl << mbslibResult.format(LongFormat) << std::endl;
                std::cout << analyticalResult.format(LongFormat) << std::endl;
            };
#endif
            analyticalResult = calculateDirectDynamics(q, dq, tau, gravity, pendulumMass, springConstant, damperConstant);

            mbs->setJointPosition(q);
            mbs->setJointVelocity(dq);
            joint1->setJointForceTorque(tau);

            mbs->doABA();

            mbslibResult = mbs->getJointAcceleration();
            //TVectorX q, TVectorX dq, TScalar tau, TScalar g, TScalar m, TScalar k, TScalar d
            //BOOST_CHECK((mbslibResult - analyticalResult).isZero());
            BOOST_CHECK(std::fabs(mbslibResult(0) - analyticalResult(0)) < 1e-10);

#ifdef ERROR_PRINTOUT
            bool error = std::fabs(mbslibResult(0) - analyticalResult(0)) > 1e-10;
            if(error) {
                printData();
                printResults();
            }
#endif

            mbs->doCrba();

            mbslibResult = mbs->getJointAcceleration();
            BOOST_CHECK(std::fabs(mbslibResult(0) - analyticalResult(0)) < 1e-10);
            //BOOST_CHECK((mbslibResult - analyticalResult).isZero());
    #ifdef ERROR_PRINTOUT
            bool error2 = std::fabs(mbslibResult(0) - analyticalResult(0)) > 1e-10;
            if(! error && error2) {
                printData();
            }
            if(error || error2) {
                printResults();
            }
    #endif
        }
        }
        }
        delete mbs;
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
    TVectorX q(1), dq(1);
    q << (3.14 / 2);
    TVectorX ddq(1);

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    TScalar mbslibResult;
    TScalar analyticalResult;

    // min, max, step
    const MMSTs _ddq                {-10.0,   10.0,   5.0};
    const MMSTs _gravity            {-10.0,   10.0,   5.0};
    const MMSTs _pendulumMass       {  5.0,   30.0,   5.0};
    const MMSTs _q_counter          {  0.0,  500.0,  25.0};
    const MMSTs _dq_counter         {  0.0,   50.0,  10.0};
    const MMSTs _springConstant     {  1.0,  151.0,   5.0};
    const MMSTs _damperConstant     {  0.0,    5.0,   0.5};

    for (TScalar gravity = _gravity.min; gravity <= _gravity.max; gravity += _gravity.step) {
    for (TScalar pendulumMass = _pendulumMass.min; pendulumMass <= _pendulumMass.max; pendulumMass += _pendulumMass.step) {
    for (TScalar springConstant = _springConstant.min; springConstant <= _springConstant.max; springConstant += _springConstant.step) {
    for (TScalar damperConstant = _damperConstant.min; damperConstant <= _damperConstant.max; damperConstant += _damperConstant.step) {
        MbsCompoundWithBuilder * mbs;

        mbs = buildModel(pendulumMass, springConstant, damperConstant, TVector3(0.0, -gravity, 0.0));
        Joint1DOF * joint1 = mbs->getJointByName("q");

        for (ddq(0) = _ddq.min; ddq(0) <= _ddq.max; ddq(0) += _ddq.step) {
        for (q(0) = _q_counter.min; q(0) <= _q_counter.max; q(0) += _q_counter.step) {
        for (dq(0) = _dq_counter.min; dq(0) <= _dq_counter.max; dq(0) += _dq_counter.step) {
#ifdef ERROR_PRINTOUT
            auto printData = [&]() {
                std::cout << "=================================" << std::endl;
                std::cout << "k " << springConstant << " d " << damperConstant << std::endl;
                std::cout << "q " << q_counter << " dq " << dq_counter << " ddq " << ddq << std::endl;
                std::cout << "m " << pendulumMass << " g " << gravity << std::endl;
            };
            auto printResults = [&]() {
                std::cout << "mbslib result: " << std::endl << mbslibResult << std::endl;
                std::cout << analyticalResult << std::endl;
            };
#endif

            mbs->setJointPosition(q);
            mbs->setJointVelocity(dq);
            mbs->setJointAcceleration(ddq);

            mbs->doRne();

            mbslibResult = joint1->getJointForceTorque();
            analyticalResult = calculateInverseDynamics(q, dq, ddq, gravity, pendulumMass, springConstant, damperConstant);

            BOOST_CHECK(std::fabs(mbslibResult - analyticalResult) < 1e-10);
#ifdef ERROR_PRINTOUT
            bool error = std::fabs(mbslibResult - analyticalResult) > 1e-10;
            if(error) {
                printData();
                printResults();
            }
#endif
        }
        }
        }
        delete mbs;
    }
    }
    }
    }
}
