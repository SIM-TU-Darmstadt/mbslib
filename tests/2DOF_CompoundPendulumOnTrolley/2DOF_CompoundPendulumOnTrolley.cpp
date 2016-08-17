#define BOOST_TEST_MODULE 2DOF_CompoundPendulumOnTrolley
#include <boost/test/included/unit_test.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>
#include <mbslib/elements/drive/PassiveSpringDamperDrive.hpp>
#include <mbslib/elements/joint/JointForceSetter.hpp>
#include <mbslib/elements/spring/model/LinearSpringModel.hpp>
#include <mbslib/utility/internalTests.hpp>
#include <stdio.h>

#define TOLERANCE 1e-9

#define TEST_CHECK_EQUAL(A, B) MBS_TEST_CHECK_EQUAL(A, B, TOLERANCE)
#define TEST_CHECK_EQUAL_RESULT(A, B, RESULT) MBS_TEST_CHECK_EQUAL_RESULT(A, B, RESULT, TOLERANCE)

using namespace mbslib;

/**
 * \brief Calculate direct kinematics
 * 		x = 0 ,		y = - q
 *
 * \param q joint position
 */
TVector3Vector calculateDirectKinematics(TVectorX q, TScalar length) {
    TVector3Vector v;
    {
        TVector3 xyCoordinates = TVector3::Zero();
        xyCoordinates(0) = q(0);
        xyCoordinates(1) = 0;
        v.push_back(xyCoordinates);
    }
    {
        TVector3 xyCoordinates = TVector3::Zero();
        xyCoordinates(0) = q(0) + length * sin(q(1));
        xyCoordinates(1) = -length * cos(q(1));
        v.push_back(xyCoordinates);
    }
    return v;
}

TVectorX calculateDirectDynamics(TVectorX q, TVectorX dq, TVectorX tau,
                                 TScalar pendulumLength,
                                 TScalar pendulumMass,
                                 TScalar gravity,
                                 TScalar k,
                                 TScalar d) {
    Eigen::VectorXd ddq(2);
    ddq.setZero();

    const double l = pendulumLength;
    const double m = pendulumMass;
    const double g = -gravity;

    const double tau1 = tau(0);
    const double tau2 = tau(1);

    const double t3 = q(1);
    const double t2 = cos(t3);
    const double t4 = dq(1);
    const double t5 = sin(t3);
    const double t6 = t2 * t2;
    const double t7 = l * l;
    const double t8 = t4 * t4;
    ddq << (l * tau1 * 4.0 - t2 * tau2 * 6.0 + d * t2 * t4 * 6.0 + k * t2 * t3 * 6.0 + m * t5 * t7 * t8 * 2.0 + g * l * m * t2 * t5 * 3.0) / (l * (m * 4.0 - m * t6 * 3.0)),
        (1.0 / t7 * (tau2 * -4.0 + d * t4 * 4.0 + k * t3 * 4.0 + l * t2 * tau1 * 2.0 + g * l * m * t5 * 2.0 + m * t2 * t5 * t7 * t8) * 3.0) / (m * (t6 * 3.0 - 4.0));

    return ddq;
}

MbsCompoundWithBuilder * buildModel(
    TScalar pendulumLength,
    TScalar pendulumMass,
    TScalar gravity,
    TScalar k,
    TScalar d) {
    mbslib::MbsCompoundWithBuilder * mbs = new mbslib::MbsCompoundWithBuilder("PendulumOnTrolley");

    const double l = pendulumLength;
    const double m = pendulumMass;

    mbs->addFixedBase();

    mbslib::TVector3 dir;
    dir << 1.0, 0.0, 0.0;
    mbs->addPrismaticJoint(dir, "q1");
    mbs->addFork();
    mbs->addEndpoint("trolley");

    dir << 0.0, 0.0, 1.0;
    RevoluteJoint * q2 = mbs->addRevoluteJoint(dir, "q2");

    dir << 0.0, -l, 0.0;
    mbs->addFixedTranslation(dir);
    mbs->addFork();
    dir << 0.0, l / 2.0, 0.0;
    mbs->addFixedTranslation(dir);

    mbslib::TMatrix3x3 I;
    I << (m * l * l) / 12.0, 0.0, 0.0,
        0.0, (m * l * l) / 12.0, 0.0,
        0.0, 0.0, (m * l * l) / 12.0;
    mbs->addEndpoint(m, I, "pendulum_com");

    mbs->addEndpoint("pendulum");

    dir << 0.0, gravity, 0.0;
    mbs->setGravitation(dir);

    mbslib::LinearSpringModel springModel(k, d);
    mbs->addSpring(springModel, *q2);
    return mbs;
}

MbsCompoundWithBuilder * buildModelWithDrive(
    TScalar pendulumLength,
    TScalar pendulumMass,
    TScalar gravity,
    TScalar k,
    TScalar d) {
    mbslib::MbsCompoundWithBuilder * mbs = new mbslib::MbsCompoundWithBuilder("PendulumOnTrolley");

    const double l = pendulumLength;
    const double m = pendulumMass;

    mbs->addFixedBase();

    mbslib::TVector3 dir;
    dir << 1.0, 0.0, 0.0;
    mbs->addPrismaticJoint(dir, "q1");
    mbs->addFork();
    mbs->addEndpoint("trolley");

    dir << 0.0, 0.0, 1.0;
    RevoluteJoint * q2 = mbs->addRevoluteJoint(dir, "q2");

    dir << 0.0, -l, 0.0;
    mbs->addFixedTranslation(dir);
    mbs->addFork();
    dir << 0.0, l / 2.0, 0.0;
    mbs->addFixedTranslation(dir);

    mbslib::TMatrix3x3 I;
    I << (m * l * l) / 12.0, 0.0, 0.0,
        0.0, (m * l * l) / 12.0, 0.0,
        0.0, 0.0, (m * l * l) / 12.0;
    mbs->addEndpoint(m, I, "pendulum_com");

    mbs->addEndpoint("pendulum");

    dir << 0.0, gravity, 0.0;
    mbs->setGravitation(dir);

    PassiveSpringDamperDrive * psdd = new PassiveSpringDamperDrive(*q2, k, d, "PSDD");
    ((MbsCompound *)mbs)->addDrive(psdd);

    return mbs;
}

BOOST_AUTO_TEST_CASE(DirectKinematics) {
    std::cout << "Test case DirectKinematics" << std::endl;
    TScalar pendulumLength = 1.0;
    TScalar pendulumMass = 5.0;
    TScalar gravity = -9.81;
    TScalar k = 10.0;
    TScalar d = 0.6;

    TVector3Vector mbslibResult;
    mbslibResult.reserve(2);
    TVector3Vector analyticalResult;
    Endpoint *ep_pendulum = nullptr, *ep_trolley = nullptr;
    MbsCompoundWithBuilder * mbs;

    TVectorX q(2);
    q.setZero();

    const MMSTs _q{-10.0, 10.0, 1.0 /* 5.0*/};
    const MMSTs _l{0.1, 10.0, 0.1 /* 5.0*/};

    for (pendulumLength = _l.min; pendulumLength <= _l.max; pendulumLength += _l.step) {
        MbsCompoundWithBuilder * mbs;
        mbs = buildModel(pendulumLength, pendulumMass, gravity, k, d);
        ep_pendulum = mbs->getEndpointByName("pendulum");
        ep_trolley = mbs->getEndpointByName("trolley");
        for (q(0) = _q.min; q(0) <= _q.max; q(0) += _q.step) {
            for (q(1) = _q.min; q(1) <= _q.max; q(1) += _q.step) {

                mbs->setJointPosition(q);
                mbs->doDirkin();

                mbslibResult[0] = ep_trolley->getCoordinateFrame().r;
                mbslibResult[1] = ep_pendulum->getCoordinateFrame().r;

                analyticalResult = calculateDirectKinematics(q, pendulumLength);

                TEST_CHECK_EQUAL(mbslibResult[0], analyticalResult[0]);
                TEST_CHECK_EQUAL(mbslibResult[1], analyticalResult[1]);
            }
        }
        delete mbs;
    }
}

TVectorX calculateInverseDynamics(TVectorX q, TVectorX dq, TVectorX ddq,
                                  TScalar pendulumLength,
                                  TScalar pendulumMass,
                                  TScalar gravity,
                                  TScalar k,
                                  TScalar d) {
    using namespace std;
    Eigen::VectorXd tau(2);
    tau.setZero();

    const double ddq1 = ddq(0);
    const double ddq2 = ddq(1);
    const double dq2 = dq(1);
    const double q2 = q(1);
    const double m = pendulumMass;
    const double l = pendulumLength;
    const double g = -gravity;

    tau << m * (ddq1 * 2.0 + ddq2 * l * cos(q2) - (dq2 * dq2) * l * sin(q2)) * (1.0 / 2.0),
        d * dq2 + k * q2 + ddq2 * (l * l) * m * (1.0 / 3.0) + ddq1 * l * m * cos(q2) * (1.0 / 2.0) + g * l * m * sin(q2) * (1.0 / 2.0);

    return tau;
}
BOOST_AUTO_TEST_CASE(DirectDynamics) {
    std::cout << "Test case DirectDynamics" << std::endl;
    TScalar pendulumLength = 1.0;
    TScalar pendulumMass = 5.0;
    TScalar gravity = -9.81;
    TScalar k = 10.0;
    TScalar d = 0.6;

    TVectorX mbslibResult(2);
    TVectorX analyticalResult(2);

    TVectorX q(2), dq(2), tau(2);
    q.setZero();
    dq.setZero();
    tau.setZero();

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    const MMSTs _q{-10.0, 10.0, 5.0};
    const MMSTs _dq{-10.0, 10.0, 5.0};
    const MMSTs _tau{-10.0, 10.0, 5.0};
    const MMSTs _k{1.0, 51.0, 10.0};
    const MMSTs _d{0.0, 10.0, 2.0};
    const MMSTs _pendulumMass{5.0, 15.0, 5.0};

    for (k = _k.min; k <= _k.max; k += _k.step) {
        for (d = _d.min; d <= _d.max; d += _d.step) {
            for (TScalar pendulumMass = _pendulumMass.min; pendulumMass <= _pendulumMass.max; pendulumMass += _pendulumMass.step) {
                MbsCompoundWithBuilder * mbs;
                mbs = buildModel(pendulumLength, pendulumMass, gravity, k, d);

                for (q(0) = _q.min; q(0) <= _q.max; q(0) += _q.step) {
                    for (q(1) = _q.min; q(1) <= _q.max; q(1) += _q.step) {
                        for (dq(0) = _dq.min; dq(0) <= _dq.max; dq(0) += _dq.step) {
                            for (dq(1) = _dq.min; dq(1) <= _dq.max; dq(1) += _dq.step) {
                                for (tau(0) = _tau.min; tau(0) <= _tau.max; tau(0) += _tau.step) {
                                    for (tau(1) = _tau.min; tau(1) <= _tau.max; tau(1) += _tau.step) {
                                        mbs->setJointPosition(q);
                                        mbs->setJointVelocity(dq);
                                        mbs->setJointForceTorque(tau);
                                        mbs->doCrba();
                                        mbslibResult = mbs->getJointAcceleration();
                                        analyticalResult = calculateDirectDynamics(q, dq, tau, pendulumLength, pendulumMass, gravity, k, d);
                                        TEST_CHECK_EQUAL(mbslibResult, analyticalResult);
                                    }
                                }
                            }
                        }
                    }
                }

                delete mbs;
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(DirectDynamicsWithDrive) {
    std::cout << "Test case DirectDynamicsWithDrive" << std::endl;
    TScalar pendulumLength = 1.0;
    TScalar pendulumMass = 5.0;
    TScalar gravity = -9.81;
    TScalar k = 10.0;
    TScalar d = 0.6;

    TVectorX mbslibResult(2);
    TVectorX analyticalResult(2);

    TVectorX q(2), dq(2), tau(2);
    q.setZero();
    dq.setZero();
    tau.setZero();

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    const MMSTs _q{-10.0, 10.0, 5.0};
    const MMSTs _dq{-10.0, 10.0, 5.0};
    const MMSTs _k{1.0, 51.0, 5.0};
    const MMSTs _d{0.0, 10.0, 2.0};
    const MMSTs _l_counter{1.0, 51.0, 10.0};

    for (k = _k.min; k <= _k.max; k += _k.step) {
        for (d = _d.min; d <= _d.max; d += _d.step) {
            for (TScalar l_counter = _l_counter.min; l_counter <= _l_counter.max; l_counter += _l_counter.step) {
                pendulumLength = l_counter / 10.0;
                MbsCompoundWithBuilder * mbs;
                mbs = buildModel(pendulumLength, pendulumMass, gravity, k, d);

                for (q(0) = _q.min; q(0) <= _q.max; q(0) += _q.step) {
                    for (q(1) = _q.min; q(1) <= _q.max; q(1) += _q.step) {
                        for (dq(0) = _dq.min; dq(0) <= _dq.max; dq(0) += _dq.step) {
                            for (dq(1) = _dq.min; dq(1) <= _dq.max; dq(1) += _dq.step) {
                                analyticalResult = calculateDirectDynamics(q, dq, tau, pendulumLength, pendulumMass, gravity, k, d);

                                mbs->setJointPosition(q);
                                mbs->setJointVelocity(dq);
                                mbs->setJointForceTorque(tau);
                                mbs->doForwardDrives();
                                mbs->doCrba();
                                mbslibResult = mbs->getJointAcceleration();
                                BOOST_CHECK((mbslibResult - analyticalResult).isZero(1e-10));

                                mbs->setJointPosition(q);
                                mbs->setJointVelocity(dq);
                                mbs->setJointForceTorque(tau);
                                mbs->doForwardDrives();
                                mbs->doABA();
                                mbslibResult = mbs->getJointAcceleration();
                                TEST_CHECK_EQUAL(mbslibResult, analyticalResult);
                            }
                        }
                    }
                }
                delete mbs;
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(InverseDynamics) {
    std::cout << "Test case InverseDynamics" << std::endl;
    TScalar pendulumLength = 1.0;
    TScalar pendulumMass = 5.0;
    TScalar gravity = -9.81;
    TScalar k = 10.0;
    TScalar d = 0.6;

    TVectorX mbslibResult(2);
    TVectorX analyticalResult(2);

    TVectorX q(2), dq(2), ddq(2);
    q.setZero();
    dq.setZero();
    ddq.setZero();

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    const MMSTs _q{-10.0, 10.0, 5.0};
    const MMSTs _dq{-10.0, 10.0, 5.0};
    const MMSTs _ddq{-10.0, 10.0, 5.0};
    const MMSTs _k{1.0, 51.0, 5.0};
    const MMSTs _d{0.0, 10.0, 2.0};
    const MMSTs _l_counter{1.0, 51.0, 5.0};

    for (k = _k.min; k <= _k.max; k += _k.step) {
        for (d = _d.min; d <= _d.max; d += _d.step) {
            for (TScalar l_counter = _l_counter.min; l_counter <= _l_counter.max; l_counter += _l_counter.step) {
                pendulumLength = l_counter / 10.0;

                MbsCompoundWithBuilder * mbs;
                mbs = buildModel(pendulumLength, pendulumMass, gravity, k, d);

                for (q(0) = _q.min; q(0) <= _q.max; q(0) += _q.step) {
                    for (q(1) = _q.min; q(1) <= _q.max; q(1) += _q.step) {
                        for (dq(0) = _dq.min; dq(0) <= _dq.max; dq(0) += _dq.step) {
                            for (dq(1) = _dq.min; dq(1) <= _dq.max; dq(1) += _dq.step) {
                                for (ddq(0) = _ddq.min; ddq(0) <= _ddq.max; ddq(0) += _ddq.step) {
                                    for (ddq(1) = _ddq.min; ddq(1) <= _ddq.max; ddq(1) += _ddq.step) {
                                        mbs->setJointPosition(q);
                                        mbs->setJointVelocity(dq);
                                        mbs->setJointAcceleration(ddq);
                                        mbs->doRne();
                                        mbslibResult = mbs->getJointForceTorque();
                                        analyticalResult = calculateInverseDynamics(q, dq, ddq, pendulumLength, pendulumMass, gravity, k, d);
                                        TEST_CHECK_EQUAL(mbslibResult, analyticalResult);
                                    }
                                }
                            }
                        }
                    }
                }
                delete mbs;
            }
        }
    }
}
