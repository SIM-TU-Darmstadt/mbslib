#define BOOST_TEST_MODULE 2DOF_ScaraManipulator
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Core>
#include <Eigen/Householder>
#include <Eigen/QR>
#include <fstream>
#include <iostream>
#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>
#include <mbslib/elements/joint/JointForceSetter.hpp>
#include <mbslib/utility/internalTests.hpp>
#include <random>
#include <stdio.h>

using namespace mbslib;

#define TOLERANCE 1e-9

#define TEST_CHECK_EQUAL(A, B) MBS_TEST_CHECK_EQUAL(A, B, TOLERANCE)
#define TEST_CHECK_EQUAL_RESULT(A, B, RESULT) MBS_TEST_CHECK_EQUAL_RESULT(A, B, RESULT, TOLERANCE)

/**
 * \brief Calculate direct kinematics
 * 		x = 0 ,		y = - q
 *
 * \param q joint position
 */
TVector3Vector calculateDirectKinematics(TVectorX q, TVectorX l) {
    TVector3Vector v;
    {
        TVector3 xyCoordinates = TVector3::Zero();
        xyCoordinates(0) = l(0) * cos(q(0));
        xyCoordinates(1) = l(0) * sin(q(0));
        v.push_back(xyCoordinates);
    }
    {
        TVector3 xyCoordinates = TVector3::Zero();
        xyCoordinates(0) = l(0) * cos(q(0)) + l(1) * cos(q(0) + q(1));
        xyCoordinates(1) = l(0) * sin(q(0)) + l(1) * sin(q(0) + q(1));
        v.push_back(xyCoordinates);
    }
    return v;
}

TVectorX calculateG(
    TVectorX q,
    TScalar g,
    TVectorX l,
    TVectorX m,
    TVector3Vector r_com) {
    TVectorX G(2);
    G(0) = m(0) * g * ((l(0) + r_com[0](0)) * cos(q(0)) - r_com[0](1) * sin(q(0))) + m(1) * g * (l(0) * sin(q(1)) * sin(q(0) + q(1)) + l(0) * cos(q(1)) * cos(q(0) + q(1)) + (r_com[1](0) + l(1)) * cos(q(0) + q(1)) - r_com[1](1) * sin(q(0) + q(1)));
    G(1) = m(1) * g * ((r_com[1](0) + l(1)) * cos(q(0) + q(1)) - r_com[1](1) * sin(q(0) + q(1)));
    return G;
}

TVectorX calculateC(
    TVectorX q,
    TVectorX dq,
    TVectorX l,
    TVectorX m,
    TVector3Vector r_com) {
    TVectorX C(2);
    C(0) = -(l(0) * pow(sin(q(0)), 2) + l(0) * pow(cos(q(0)), 2)) * (m(1) * sin(q(1)) * (pow(dq(0) + dq(1), 2) * (l(1) * pow(sin(q(1)), 2) + l(1) * pow(cos(q(1)), 2)) + r_com[1](0) * pow(dq(0) + dq(1), 2) + (dq(0) * dq(0)) * cos(q(1)) * (l(0) * pow(sin(q(0)), 2) + l(0) * pow(cos(q(0)), 2))) + m(1) * cos(q(1)) * (r_com[1](1) * pow(dq(0) + dq(1), 2) - (dq(0) * dq(0)) * sin(q(1)) * (l(0) * pow(sin(q(0)), 2) + l(0) * pow(cos(q(0)), 2)))) - m(1) * (r_com[1](1) * pow(dq(0) + dq(1), 2) - (dq(0) * dq(0)) * sin(q(1)) * (l(0) * pow(sin(q(0)), 2) + l(0) * pow(cos(q(0)), 2))) * (r_com[1](0) + l(1) * pow(sin(q(1)), 2) + l(1) * pow(cos(q(1)), 2)) + m(0) * r_com[0](1) * ((dq(0) * dq(0)) * (l(0) * pow(sin(q(0)), 2) + l(0) * pow(cos(q(0)), 2)) + (dq(0) * dq(0)) * r_com[0](0)) + m(1) * r_com[1](1) * (pow(dq(0) + dq(1), 2) * (l(1) * pow(sin(q(1)), 2) + l(1) * pow(cos(q(1)), 2)) + r_com[1](0) * pow(dq(0) + dq(1), 2) + (dq(0) * dq(0)) * cos(q(1)) * (l(0) * pow(sin(q(0)), 2) + l(0) * pow(cos(q(0)), 2))) - m(0) * (dq(0) * dq(0)) * r_com[0](1) * (r_com[0](0) + l(0) * pow(sin(q(0)), 2) + l(0) * pow(cos(q(0)), 2));
    C(1) = m(1) * l(0) * ((r_com[1](0) + l(1)) * sin(q(1)) + r_com[1](1) * cos(q(1))) * pow(dq(0), 2);
    return C;
}

TMatrixX calculateM(
    TVectorX q,
    TVectorX l,
    TVectorX m,
    TVector3Vector r_com,
    TMatrix3x3Vector I_com) {
    TMatrixX M(2, 2);

    M(0, 0) = I_com[0](2, 2) + I_com[1](2, 2) + m(0) * pow(r_com[0](0) + l(0) * pow(sin(q(0)), 2) + l(0) * pow(cos(q(0)), 2), 2) + (l(0) * pow(sin(q(0)), 2) + l(0) * pow(cos(q(0)), 2)) * (m(1) * cos(q(1)) * (r_com[1](0) + l(1) * pow(sin(q(1)), 2) + cos(q(1)) * (l(0) * pow(sin(q(0)), 2) + l(0) * pow(cos(q(0)), 2)) + l(1) * pow(cos(q(1)), 2)) - m(1) * sin(q(1)) * (r_com[1](1) - sin(q(1)) * (l(0) * pow(sin(q(0)), 2) + l(0) * pow(cos(q(0)), 2)))) + m(0) * (r_com[0](1) * r_com[0](1)) + m(1) * (r_com[1](0) + l(1) * pow(sin(q(1)), 2) + l(1) * pow(cos(q(1)), 2)) * (r_com[1](0) + l(1) * pow(sin(q(1)), 2) + cos(q(1)) * (l(0) * pow(sin(q(0)), 2) + l(0) * pow(cos(q(0)), 2)) + l(1) * pow(cos(q(1)), 2)) + m(1) * r_com[1](1) * (r_com[1](1) - sin(q(1)) * (l(0) * pow(sin(q(0)), 2) + l(0) * pow(cos(q(0)), 2)));
    M(1, 0) = M(0, 1) = m(1) * (l(0) * cos(q(1)) * (r_com[1](0) + l(1)) - l(0) * sin(q(1)) * r_com[1](1) + pow(r_com[1](0) + l(1), 2) + pow(r_com[1](1), 2)) + I_com[1](2, 2);
    M(1, 1) = m(1) * (pow(r_com[1](0) + l(1), 2) + pow(r_com[1](1), 2)) + I_com[1](2, 2);
    return M;
}

/**
 * @brief Calculate direct dynamics
 * \param q joint position
 * \param dq joint velocity
 * \param tau joint torque
 * \param g g
 * \param l length
 * \param m mass
 * \param r_com
 * \param I_com
 * @return
 */

TVectorX calculateDirectDynamics(
    TVectorX q,
    TVectorX dq,
    TVectorX tau,
    TScalar g,
    TVectorX l,
    TVectorX m,
    TVector3Vector r_com,
    TMatrix3x3Vector I_com) {
    TVectorX ddq(2);

    TMatrixX M(2, 2);
    TVectorX C(2);
    TVectorX G(2);
    M = calculateM(q, l, m, r_com, I_com);
    C = calculateC(q, dq, l, m, r_com);
    G = calculateG(q, g, l, m, r_com);

    ddq = M.householderQr().solve(tau - C - G);
    return ddq;
}

TVectorX calculateInverseDynamics(
    TVectorX q,
    TVectorX dq,
    TVectorX ddq,
    TScalar g,
    TVectorX l,
    TVectorX m,
    TVector3Vector r_com,
    TMatrix3x3Vector I_com) {
    TVectorX tau(2);

    TMatrixX M(2, 2);
    TVectorX C(2);
    TVectorX G(2);
    M = calculateM(q, l, m, r_com, I_com);
    C = calculateC(q, dq, l, m, r_com);
    G = calculateG(q, g, l, m, r_com);

    tau = M * ddq + C + G;
    return tau;
}
MbsCompoundWithBuilder * buildModel(
    TVectorX l,
    TVectorX m,
    TScalar g,
    TVector3Vector r_com,
    TMatrix3x3Vector I_com) {
    mbslib::MbsCompoundWithBuilder * mbs = new mbslib::MbsCompoundWithBuilder("PendulumOnTrolley");

    mbs->addFixedBase();

    mbs->addRevoluteJointZ(0.0, "q1");
    mbs->addRigidLink(TVector3(l(0), 0.0, 0.0), r_com[0], m(0), I_com[0], "l1");
    mbs->addRevoluteJointZ(0.0, "q2");
    mbs->addRigidLink(TVector3(l(1), 0.0, 0.0), r_com[1], m(1), I_com[1], "l2");
    mbs->addEndpoint("endpoint");

    mbs->setGravitation(TVector3(0.0, -g, 0.0));

    return mbs;
}

#ifdef CHECK_MCG
BOOST_AUTO_TEST_CASE(CalculateMCG) {
    std::cout << "Test case CalculateM" << std::endl;
    TVectorX l(2);
    TVectorX m(2);

    TScalar g = -9.81;

    TVector3Vector r_com;
    r_com.assign(2, TVector3::Zero());

    TMatrix3x3Vector I_com;
    I_com.assign(2, TMatrix3x3::Zero());

    TVectorX mbslibResult(2);
    TVectorX analyticalResult(2);

    TVectorX q(2), dq(2), tau(2);
    q.setZero();
    dq.setZero();
    tau.setZero();

    TVectorX zero = TVectorX::Zero(2);

    TMatrixX M(2, 2), M2(2, 2);
    TVectorX C(2), C2(2);
    TVectorX G(2), G2(2);

    const MMSTs _q{-10.0, 10.0, 1.0 /* 5.0*/};
    const MMSTs _l{1.0, 51.0, 10.0};
    const MMSTs _m{1.0, 15.0, 1.0};
    const MMSTs _r{-10.0, 10.0, 5.0};
    const MMSTs _I{0.0, 1.0, 0.1};

    for (l(0) = _l.min; l(0) <= _l.max; l(0) += _l.step) {
        for (l(1) = _l.min; l(1) <= _l.max; l(1) += _l.step) {
            for (m(0) = _m.min; m(0) <= _m.max; m(0) += _m.step) {
                for (m(1) = _m.min; m(1) <= _m.max; m(1) += _m.step) {
                    for (I_com[0](2, 2) = _I.min; I_com[0](2, 2) <= _I.max; I_com[0](2, 2) += _I.step) {
                        for (I_com[1](2, 2) = _I.min; I_com[1](2, 2) <= _I.max; I_com[1](2, 2) += _I.step) {
                            for (r_com[0](0) = _r.min; r_com[0](0) <= _r.max; r_com[0](0) += _r.step) {
                                for (r_com[0](1) = _r.min; r_com[0](1) <= _r.max; r_com[0](1) += _r.step) {
                                    for (r_com[1](0) = _r.min; r_com[1](0) <= _r.max; r_com[1](0) += _r.step) {
                                        for (r_com[1](1) = _r.min; r_com[1](1) <= _r.max; r_com[1](1) += _r.step) {
                                            MbsCompoundWithBuilder * mbs;
                                            mbs = buildModel(l, m, g, r_com, I_com);
                                            for (q(0) = _q.min; q(0) <= _q.max; q(0) += _q.step) {
                                                for (q(1) = _q.min; q(1) <= _q.max; q(1) += _q.step) {
                                                    for (dq(0) = _q.min; dq(0) <= _q.max; dq(0) += _q.step) {
                                                        for (dq(1) = _q.min; dq(1) <= _q.max; dq(1) += _q.step) {
                                                            mbs->setJointAcceleration(zero);

                                                            M = calculateM(q, l, m, r_com, I_com);
                                                            mbs->setJointPosition(q);
                                                            M2 = mbs->calculateMassMatrix2();
                                                            if (!(M - M2).isZero(1e-10)) {
                                                                std::cout << "M: " << std::endl;
                                                                std::cout << M << std::endl;
                                                                std::cout << M2 << std::endl;
                                                            }

                                                            mbs->setJointVelocity(dq);
                                                            C = calculateC(q, dq, l, m, r_com);
                                                            C2 = mbs->calculateCoriolisForceInJoints();
                                                            if (!(C - C2).isZero(1e-10)) {
                                                                std::cout << "C: " << std::endl;
                                                                std::cout << C.transpose() << std::endl;
                                                                std::cout << C2.transpose() << std::endl;
                                                            }

                                                            mbs->setJointVelocity(zero);
                                                            G = calculateG(q, g, l, m, r_com);
                                                            G2 = mbs->calculateGravitationVectorInJoints();
                                                            if (!(G - G2).isZero(1e-10)) {
                                                                std::cout << "G: " << std::endl;
                                                                std::cout << G.transpose() << std::endl;
                                                                std::cout << G2.transpose() << std::endl;
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
                        }
                    }
                }
            }
        }
    }
}
#endif
BOOST_AUTO_TEST_CASE(DirectKinematics) {
    std::cout << "Test case DirectKinematics" << std::endl;
    TVectorX l(2);
    TVectorX m(2);
    m.setOnes();

    TScalar g = -9.81;

    TVector3Vector mbslibResult;
    mbslibResult.reserve(2);
    TVector3Vector analyticalResult;
    MbsCompoundWithBuilder * mbs;

    TVector3Vector r_com;
    TMatrix3x3Vector I_com;
    r_com.push_back(TVector3::Zero());
    r_com.push_back(TVector3::Zero());
    I_com.push_back(TMatrix3x3::Zero());
    I_com.push_back(TMatrix3x3::Zero());

    TVectorX q(2);
    q.setZero();

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    const MMSTs _q{-10.0, 10.0, 1.0 /* 5.0*/};
    const MMSTs _l{1.0, 10.0, 1.0 /* 5.0*/};

    for (l(0) = _l.min; l(0) <= _l.max; l(0) += _l.step) {
        for (l(1) = _l.min; l(1) <= _l.max; l(1) += _l.step) {
            MbsCompoundWithBuilder * mbs;
            mbs = buildModel(l, m, g, r_com, I_com);
            mbslib::Joint * j2 = mbs->getJointByName("q2");
            auto ep = mbs->getEndpointByName("endpoint");
            for (q(0) = _q.min; q(0) <= _q.max; q(0) += _q.step) {
                for (q(1) = _q.min; q(1) <= _q.max; q(1) += _q.step) {
                    auto printResults = [&]() {
                        std::cout << "mbslib result: "
                                  << std::endl
                                  << mbslibResult[0].transpose().format(LongFormat)
                                  << std::endl
                                  << mbslibResult[1].transpose().format(LongFormat)
                                  << std::endl
                                  << analyticalResult[0].transpose().format(LongFormat)
                                  << std::endl
                                  << analyticalResult[1].transpose().format(LongFormat)
                                  << std::endl;
                    };

                    mbs->setJointPosition(q);
                    mbs->doDirkin();

                    mbslibResult[0] = j2->getCoordinateFrame().r;
                    mbslibResult[1] = ep->getCoordinateFrame().r;

                    analyticalResult = calculateDirectKinematics(q, l);

                    TEST_CHECK_EQUAL(mbslibResult[0], analyticalResult[0]);

                    TEST_CHECK_EQUAL(mbslibResult[1], analyticalResult[1]);

                    //BOOST_CHECK((mbslibResult[0] - analyticalResult[0]).isZero());
                    //BOOST_CHECK((mbslibResult[1] - analyticalResult[1]).isZero());
                    //        printResults();
                }
            }
            delete mbs;
        }
    }
}

BOOST_AUTO_TEST_CASE(DirectDynamicsRandom) {
    std::cout << "Test case DirectDynamics" << std::endl;
    TVectorX l(2);
    TVectorX m(2);

    TScalar g = -9.81;

    TVector3Vector r_com;
    r_com.assign(2, TVector3::Zero());

    TMatrix3x3Vector I_com;
    I_com.assign(2, TMatrix3x3::Zero());

    TVectorX mbslibResult(2);
    TVectorX analyticalResult(2);

    TVectorX q(2), dq(2), tau(2);
    q.setZero();
    dq.setZero();
    tau.setZero();

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    RigidBodyDescription rbd1, rbd2;

    const MMSTs _q{-10.0, 10.0, 5.0};
    const MMSTs _dq{-10.0, 10.0, 5.0};
    const MMSTs _tau{-10.0, 10.0, 5.0};
    const MMSTs _l{0.1, 10.1, 2.5};
    const MMSTs _m{0.1, 10.1, 2.5};
    const MMSTs _r{-0.0, 1.0, 0.5};
    const MMSTs _I{0.0, 1.0, 0.25};

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<> _q_dis(_q.min, _q.max);
    std::uniform_real_distribution<> _dq_dis(_dq.min, _dq.max);
    std::uniform_real_distribution<> _tau_dis(_tau.min, _tau.max);
    std::uniform_real_distribution<> _l_dis(_l.min, _l.max);
    std::uniform_real_distribution<> _m_dis(_m.min, _m.max);
    std::uniform_real_distribution<> _r_dis(_r.min, _r.max);
    std::uniform_real_distribution<> _I_dis(_I.min, _I.max);

    bool result;

    const size_t param_loops = 1e3,
                 value_loops = 5e3,
                 total_loops = param_loops * value_loops;
    for (size_t i = 0; i < param_loops; ++i) {
        l(0) = _l_dis(gen);
        l(1) = _l_dis(gen);
        m(0) = _m_dis(gen);
        m(1) = _m_dis(gen);
        I_com[0](2, 2) = _I_dis(gen);
        I_com[1](2, 2) = _I_dis(gen);
        r_com[0](0) = _r_dis(gen);
        r_com[1](0) = _r_dis(gen);
        r_com[0](1) = _r_dis(gen);
        r_com[1](1) = _r_dis(gen);

        MbsCompoundWithBuilder * mbs = nullptr;
        mbs = buildModel(l, m, g, r_com, I_com);
        for (size_t j = 0; j < value_loops; ++j) {
            q(0) = _q_dis(gen);
            q(1) = _q_dis(gen);
            dq(0) = _dq_dis(gen);
            dq(1) = _dq_dis(gen);
            tau(0) = _tau_dis(gen);
            tau(1) = _tau_dis(gen);

            auto printResults = [&]() {
                std::cout << "q "
                          << q.transpose()
                          << " dq "
                          << dq.transpose()
                          << " tau "
                          << tau.transpose()
                          << std::endl;
                std::cout << "l "
                          << l.transpose()
                          << " m "
                          << m.transpose()
                          << " r_com[0] "
                          << r_com[0].transpose()
                          << " r_com[1] "
                          << r_com[1].transpose()
                          << std::endl;
                std::cout << "mbslib result: " << std::endl
                          << mbslibResult.transpose().format(LongFormat) << std::endl;
                std::cout << analyticalResult.transpose().format(LongFormat) << std::endl;
            };

            analyticalResult = calculateDirectDynamics(q, dq, tau, g, l, m, r_com, I_com);

            mbs->setJointPosition(q);
            mbs->setJointVelocity(dq);
            mbs->setJointForceTorque(tau);

            mbs->doCrba();
            mbslibResult = mbs->getJointAcceleration();
            TEST_CHECK_EQUAL_RESULT(mbslibResult, analyticalResult, result);

            if (!result)
                printResults();

            mbs->doABA();
            mbslibResult = mbs->getJointAcceleration();
            TEST_CHECK_EQUAL_RESULT(mbslibResult, analyticalResult, result);

            if (!result)
                printResults();
        }
        delete mbs;
        mbs = nullptr;
    }
}

BOOST_AUTO_TEST_CASE(InverseDynamicsRandom) {
    std::cout << "Test case InverseDynamics" << std::endl;
    TVectorX l(2);
    TVectorX m(2);

    TScalar g = -9.81;

    TVector3Vector r_com;
    r_com.assign(2, TVector3::Zero());

    TMatrix3x3Vector I_com;
    I_com.assign(2, TMatrix3x3::Zero());

    TVectorX mbslibResult(2);
    TVectorX analyticalResult(2);

    TVectorX q(2), dq(2), ddq(2);
    q.setZero();
    dq.setZero();
    ddq.setZero();

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    RigidBodyDescription rbd1, rbd2;

    const MMSTs _q{-10.0, 10.0, 5.0};
    const MMSTs _dq{-10.0, 10.0, 5.0};
    const MMSTs _ddq{-10.0, 10.0, 5.0};
    const MMSTs _l{0.1, 10.1, 2.5};
    const MMSTs _m{0.1, 10.1, 2.5};
    const MMSTs _r{-0.0, 1.0, 0.5};
    const MMSTs _I{0.0, 1.0, 0.25};

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<> _q_dis(_q.min, _q.max);
    std::uniform_real_distribution<> _dq_dis(_dq.min, _dq.max);
    std::uniform_real_distribution<> _ddq_dis(_ddq.min, _ddq.max);
    std::uniform_real_distribution<> _l_dis(_l.min, _l.max);
    std::uniform_real_distribution<> _m_dis(_m.min, _m.max);
    std::uniform_real_distribution<> _r_dis(_r.min, _r.max);
    std::uniform_real_distribution<> _I_dis(_I.min, _I.max);

    bool result;

    const size_t param_loops = 1e3,
                 value_loops = 5e3,
                 total_loops = param_loops * value_loops;
    for (size_t i = 0; i < param_loops; ++i) {
        l(0) = _l_dis(gen);
        l(1) = _l_dis(gen);
        m(0) = _m_dis(gen);
        m(1) = _m_dis(gen);
        I_com[0](2, 2) = _I_dis(gen);
        I_com[1](2, 2) = _I_dis(gen);
        r_com[0](0) = _r_dis(gen);
        r_com[1](0) = _r_dis(gen);
        r_com[0](1) = _r_dis(gen);
        r_com[1](1) = _r_dis(gen);

        MbsCompoundWithBuilder * mbs = nullptr;
        mbs = buildModel(l, m, g, r_com, I_com);
        for (size_t j = 0; j < value_loops; ++j) {
            q(0) = _q_dis(gen);
            q(1) = _q_dis(gen);
            dq(0) = _dq_dis(gen);
            dq(1) = _dq_dis(gen);
            ddq(0) = _ddq_dis(gen);
            ddq(1) = _ddq_dis(gen);

            auto printResults = [&]() {
                std::cout << "q "
                          << q.transpose()
                          << " dq "
                          << dq.transpose()
                          << " ddq "
                          << ddq.transpose()
                          << std::endl;
                std::cout << "l "
                          << l.transpose()
                          << " m "
                          << m.transpose()
                          << " r_com[0] "
                          << r_com[0].transpose()
                          << " r_com[1] "
                          << r_com[1].transpose()
                          << std::endl;
                std::cout << "mbslib result: " << std::endl
                          << mbslibResult.transpose().format(LongFormat) << std::endl;
                std::cout << analyticalResult.transpose().format(LongFormat) << std::endl;
            };

            analyticalResult = calculateInverseDynamics(q, dq, ddq, g, l, m, r_com, I_com);

            mbs->setJointPosition(q);
            mbs->setJointVelocity(dq);
            mbs->setJointAcceleration(ddq);

            mbs->doRne();
            mbslibResult = mbs->getJointForceTorque();
            TEST_CHECK_EQUAL_RESULT(mbslibResult, analyticalResult, result);

            if (!result)
                printResults();
        }
        delete mbs;
        mbs = nullptr;
    }
}

BOOST_AUTO_TEST_CASE(DirectDynamics) {
    std::cout << "Test case DirectDynamics" << std::endl;
    TVectorX l(2);
    TVectorX m(2);

    TScalar g = -9.81;

    TVector3Vector r_com;
    r_com.assign(2, TVector3::Zero());

    TMatrix3x3Vector I_com;
    I_com.assign(2, TMatrix3x3::Zero());

    TVectorX mbslibResult(2);
    TVectorX analyticalResult(2);

    TVectorX q(2), dq(2), tau(2);
    q.setZero();
    dq.setZero();
    tau.setZero();

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    RigidBodyDescription rbd1, rbd2;

    const MMSTs _q{-4.0, 4.0, 4.0};
    const MMSTs _dq{-5.0, 5.0, 5.0};
    const MMSTs _tau{-6.0, 6.0, 6.0};
    const MMSTs _l{0.1, 5.1, 2.5};
    const MMSTs _m{0.1, 5.1, 2.5};
    const MMSTs _r{-0.5, 0.5, 0.5};
    const MMSTs _I{0.0, 1.0, 0.5};

    bool result;

    for (l(0) = _l.min; l(0) <= _l.max; l(0) += _l.step) {
        for (l(1) = _l.min; l(1) <= _l.max; l(1) += _l.step) {
            MbsCompoundWithBuilder * mbs = nullptr;
            RigidLink * link1 = nullptr;
            RigidLink * link2 = nullptr;
            for (m(0) = _m.min; m(0) <= _m.max; m(0) += _m.step) {
                rbd1.m = m(0);
                for (m(1) = _m.min; m(1) <= _m.max; m(1) += _m.step) {
                    rbd2.m = m(1);
                    for (I_com[0](2, 2) = _I.min; I_com[0](2, 2) <= _I.max; I_com[0](2, 2) += _I.step) {
                        rbd1.I = I_com[0];
                        for (r_com[0](0) = _r.min; r_com[0](0) <= _r.max; r_com[0](0) += _r.step) {
                            for (r_com[0](1) = _r.min; r_com[0](1) <= _r.max; r_com[0](1) += _r.step) {
                                rbd1.com = r_com[0];
                                for (I_com[1](2, 2) = _I.min; I_com[1](2, 2) <= _I.max; I_com[1](2, 2) += _I.step) {
                                    rbd2.I = I_com[1];
                                    for (r_com[1](0) = _r.min; r_com[1](0) <= _r.max; r_com[1](0) += _r.step) {
                                        for (r_com[1](1) = _r.min; r_com[1](1) <= _r.max; r_com[1](1) += _r.step) {
                                            rbd2.com = r_com[1];
                                            if (mbs == nullptr) {
                                                mbs = buildModel(l, m, g, r_com, I_com);
                                                link1 = (RigidLink *)(mbs->getElementByName("l1"));
                                                link2 = (RigidLink *)(mbs->getElementByName("l2"));
                                                rbd1.r = link1->getFixedRelativePosition();
                                                rbd2.r = link2->getFixedRelativePosition();
                                            } else {
                                                (*link1) = rbd1;
                                                (*link2) = rbd2;
                                            }

                                            for (q(0) = _q.min; q(0) <= _q.max; q(0) += _q.step) {
                                                for (q(1) = _q.min; q(1) <= _q.max; q(1) += _q.step) {
                                                    for (dq(0) = _dq.min; dq(0) <= _dq.max; dq(0) += _dq.step) {
                                                        for (dq(1) = _dq.min; dq(1) <= _dq.max; dq(1) += _dq.step) {
                                                            for (tau(0) = _tau.min; tau(0) <= _tau.max; tau(0) += _tau.step) {
                                                                for (tau(1) = _tau.min; tau(1) <= _tau.max; tau(1) += _tau.step) {

                                                                    auto printResults = [&]() {
                                                                        std::cout << "q "
                                                                                  << q.transpose()
                                                                                  << " dq "
                                                                                  << dq.transpose()
                                                                                  << " tau "
                                                                                  << tau.transpose()
                                                                                  << std::endl;
                                                                        std::cout << "l "
                                                                                  << l.transpose()
                                                                                  << " m "
                                                                                  << m.transpose()
                                                                                  << " r_com[0] "
                                                                                  << r_com[0].transpose()
                                                                                  << " r_com[1] "
                                                                                  << r_com[1].transpose()
                                                                                  << std::endl;
                                                                        std::cout << "mbslib result: " << std::endl
                                                                                  << mbslibResult.transpose().format(LongFormat) << std::endl;
                                                                        std::cout << analyticalResult.transpose().format(LongFormat) << std::endl;
                                                                    };

                                                                    analyticalResult = calculateDirectDynamics(q, dq, tau, g, l, m, r_com, I_com);

                                                                    mbs->setJointPosition(q);
                                                                    mbs->setJointVelocity(dq);
                                                                    mbs->setJointForceTorque(tau);

                                                                    mbs->doCrba();
                                                                    mbslibResult = mbs->getJointAcceleration();
                                                                    TEST_CHECK_EQUAL_RESULT(mbslibResult, analyticalResult, result);

                                                                    if (!result)
                                                                        printResults();

                                                                    mbs->doABA();
                                                                    mbslibResult = mbs->getJointAcceleration();
                                                                    TEST_CHECK_EQUAL_RESULT(mbslibResult, analyticalResult, result);

                                                                    if (!result)
                                                                        printResults();
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            delete mbs;
            mbs = nullptr;
        }
    }
}
BOOST_AUTO_TEST_CASE(InverseDynamics) {
    std::cout << "Test case InverseDynamics" << std::endl;
    TVectorX l(2);
    TVectorX m(2);

    TScalar g = -9.81;

    TVector3Vector r_com;
    r_com.assign(2, TVector3::Zero());

    TMatrix3x3Vector I_com;
    I_com.assign(2, TMatrix3x3::Zero());

    TVectorX mbslibResult(2);
    TVectorX analyticalResult(2);

    TVectorX q(2), dq(2), ddq(2);
    q.setZero();
    dq.setZero();
    ddq.setZero();

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    RigidBodyDescription rbd1, rbd2;

    const MMSTs _q{-4.0, 4.0, 4.0};
    const MMSTs _dq{-5.0, 5.0, 5.0};
    const MMSTs _ddq{-6.0, 6.0, 6.0};
    const MMSTs _l{0.1, 5.1, 2.5};
    const MMSTs _m{0.1, 5.1, 2.5};
    const MMSTs _r{-0.5, 0.5, 0.5};
    const MMSTs _I{0.0, 1.0, 0.5};

    bool result;

    for (l(0) = _l.min; l(0) <= _l.max; l(0) += _l.step) {
        for (l(1) = _l.min; l(1) <= _l.max; l(1) += _l.step) {
            MbsCompoundWithBuilder * mbs = nullptr;
            RigidLink * link1 = nullptr;
            RigidLink * link2 = nullptr;
            for (m(0) = _m.min; m(0) <= _m.max; m(0) += _m.step) {
                rbd1.m = m(0);
                for (m(1) = _m.min; m(1) <= _m.max; m(1) += _m.step) {
                    rbd2.m = m(1);
                    for (I_com[0](2, 2) = _I.min; I_com[0](2, 2) <= _I.max; I_com[0](2, 2) += _I.step) {
                        rbd1.I = I_com[0];
                        for (r_com[0](0) = _r.min; r_com[0](0) <= _r.max; r_com[0](0) += _r.step) {
                            for (r_com[0](1) = _r.min; r_com[0](1) <= _r.max; r_com[0](1) += _r.step) {
                                rbd1.com = r_com[0];
                                for (I_com[1](2, 2) = _I.min; I_com[1](2, 2) <= _I.max; I_com[1](2, 2) += _I.step) {
                                    rbd2.I = I_com[1];
                                    for (r_com[1](0) = _r.min; r_com[1](0) <= _r.max; r_com[1](0) += _r.step) {
                                        for (r_com[1](1) = _r.min; r_com[1](1) <= _r.max; r_com[1](1) += _r.step) {
                                            rbd2.com = r_com[1];
                                            if (mbs == nullptr) {
                                                mbs = buildModel(l, m, g, r_com, I_com);
                                                link1 = (RigidLink *)(mbs->getElementByName("l1"));
                                                link2 = (RigidLink *)(mbs->getElementByName("l2"));
                                                rbd1.r = link1->getFixedRelativePosition();
                                                rbd2.r = link2->getFixedRelativePosition();
                                            } else {
                                                (*link1) = rbd1;
                                                (*link2) = rbd2;
                                            }

                                            for (q(0) = _q.min; q(0) <= _q.max; q(0) += _q.step) {
                                                for (q(1) = _q.min; q(1) <= _q.max; q(1) += _q.step) {
                                                    for (dq(0) = _dq.min; dq(0) <= _dq.max; dq(0) += _dq.step) {
                                                        for (dq(1) = _dq.min; dq(1) <= _dq.max; dq(1) += _dq.step) {
                                                            for (ddq(0) = _ddq.min; ddq(0) <= _ddq.max; ddq(0) += _ddq.step) {
                                                                for (ddq(1) = _ddq.min; ddq(1) <= _ddq.max; ddq(1) += _ddq.step) {

                                                                    auto printResults = [&]() {
                                                                        std::cout << "q "
                                                                                  << q.transpose()
                                                                                  << " dq "
                                                                                  << dq.transpose()
                                                                                  << " ddq "
                                                                                  << ddq.transpose()
                                                                                  << std::endl;
                                                                        std::cout << "l "
                                                                                  << l.transpose()
                                                                                  << " m "
                                                                                  << m.transpose()
                                                                                  << " r_com[0] "
                                                                                  << r_com[0].transpose()
                                                                                  << " r_com[1] "
                                                                                  << r_com[1].transpose()
                                                                                  << std::endl;
                                                                        std::cout << "mbslib result: " << std::endl
                                                                                  << mbslibResult.transpose().format(LongFormat) << std::endl;
                                                                        std::cout << analyticalResult.transpose().format(LongFormat) << std::endl;
                                                                    };

                                                                    mbs->setJointPosition(q);
                                                                    mbs->setJointVelocity(dq);
                                                                    mbs->setJointAcceleration(ddq);
                                                                    mbs->doRne();
                                                                    mbslibResult = mbs->getJointForceTorque();
                                                                    analyticalResult = calculateInverseDynamics(q, dq, ddq, g, l, m, r_com, I_com);
                                                                    TEST_CHECK_EQUAL_RESULT(mbslibResult, analyticalResult, result);

                                                                    if (!result)
                                                                        printResults();
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            delete mbs;
            mbs = nullptr;
            link1 = nullptr;
            link2 = nullptr;
        }
    }
}
