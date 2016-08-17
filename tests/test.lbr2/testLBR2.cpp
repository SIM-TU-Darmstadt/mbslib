#define BOOST_TEST_MODULE mbslibLBR2
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Core>
#include <cmath>
#include <fstream>
#include <iostream>
#include <math.h>
#include <vector>

#include <boost/numeric/odeint.hpp>
#include <mbslib/utility/internalTests.hpp>

#include <model.lbr2/LBR2ModelAnalytic.hpp>
#include <model.lbr2/LBR2ModelMbslib.hpp>
#include <random>

using namespace mbslib;
#define TOLERANCE 1e-6

#define TEST_CHECK_EQUAL(A, B) MBS_TEST_CHECK_EQUAL(A, B, TOLERANCE)
#define TEST_CHECK_EQUAL_RESULT(A, B, RESULT) MBS_TEST_CHECK_EQUAL_RESULT(A, B, RESULT, TOLERANCE)

BOOST_AUTO_TEST_CASE(CalculateGravTorque) {
    std::cout << "Test case CalculateGravTorque" << std::endl;
    using namespace mbslib;
    LBR2ModelAnalytic * modelAnalytic = new LBR2ModelAnalytic;
    LBR2ModelMbslib * modelMbslib = new LBR2ModelMbslib;
    LBR2ModelMbslib * modelMbslib2 = new LBR2ModelMbslib(LBR2ModelMbslib::Model::ModelDH2);
    modelMbslib->algorithm = LBR2ModelMbslib::CRBA;
    modelMbslib2->algorithm = LBR2ModelMbslib::CRBA;
    const size_t dof = 7;

    double mass = 0.0;
    mbslib::TVector3 rl;
    rl.setZero();

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    const MMSTs _q{-10.0, 10.0, 5.0};
    const MMSTs _dq{-10.0, 10.0, 5.0};
    const MMSTs _ddq{-10.0, 10.0, 5.0};

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<> _q_dis(_q.min, _q.max);

    mbslib::TVectorX q(dof), zero = mbslib::TVectorX::Zero(dof);
    mbslib::TVectorX mbslibResult(dof), analyticalResult(dof);

    bool result;

    const size_t value_loops = 1e6,
                 total_loops = value_loops;

    for (size_t j = 0; j < value_loops; ++j) {
        for (size_t i = 0; i < dof; ++i) {
            q(i) = _q_dis(gen);
        }

        auto printResults = [&]() {
            std::cout << "q "
                      << q.transpose()
                      << std::endl;
            std::cout << "mbslib result: " << std::endl
                      << mbslibResult.transpose().format(LongFormat) << std::endl;
            std::cout << analyticalResult.transpose().format(LongFormat) << std::endl;
        };

        analyticalResult = modelAnalytic->gravityTorque(q);

        mbslibResult = modelMbslib->gravityTorque(q);
        TEST_CHECK_EQUAL_RESULT(analyticalResult, mbslibResult, result);
        if (!result)
            printResults();

        mbslibResult = modelMbslib2->gravityTorque(q);
        TEST_CHECK_EQUAL_RESULT(analyticalResult, mbslibResult, result);
        if (!result)
            printResults();
    }

    delete modelAnalytic;
    delete modelMbslib;
}

BOOST_AUTO_TEST_CASE(MassMatrix) {
    std::cout << "Test case MassMatrix" << std::endl;
    using namespace mbslib;
    LBR2ModelAnalytic * modelAnalytic = new LBR2ModelAnalytic;
    LBR2ModelMbslib * modelMbslib = new LBR2ModelMbslib;
    LBR2ModelMbslib * modelMbslib2 = new LBR2ModelMbslib(LBR2ModelMbslib::Model::ModelDH2);
    modelMbslib->algorithm = LBR2ModelMbslib::CRBA;
    modelMbslib2->algorithm = LBR2ModelMbslib::CRBA;
    const size_t dof = 7;

    double mass = 0.0;
    mbslib::TVector3 rl;
    rl.setZero();

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    const MMSTs _q{-10.0, 10.0, 5.0};
    const MMSTs _dq{-10.0, 10.0, 5.0};
    const MMSTs _ddq{-10.0, 10.0, 5.0};

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<> _q_dis(_q.min, _q.max);

    mbslib::TVectorX q(dof), zero = mbslib::TVectorX::Zero(dof);
    mbslib::TMatrixX mbslibResult(dof, dof), analyticalResult(dof, dof);

    bool result;

    const size_t value_loops = 1e6,
                 total_loops = value_loops;

    for (size_t j = 0; j < value_loops; ++j) {
        for (size_t i = 0; i < dof; ++i) {
            q(i) = _q_dis(gen);
        }

        auto printResults = [&]() {
            std::cout << "q "
                      << q.transpose()
                      << std::endl;
            std::cout << "mbslib result: " << std::endl
                      << mbslibResult.transpose().format(LongFormat) << std::endl;
            std::cout << analyticalResult.transpose().format(LongFormat) << std::endl;
        };

        analyticalResult = modelAnalytic->massMatrix(q);
        mbslibResult = modelMbslib->massMatrix(q);
        TEST_CHECK_EQUAL_RESULT(analyticalResult, mbslibResult, result);

        if (!result)
            printResults();

        mbslibResult = modelMbslib2->massMatrix(q);
        TEST_CHECK_EQUAL_RESULT(analyticalResult, mbslibResult, result);

        if (!result)
            printResults();
    }

    delete modelAnalytic;
    delete modelMbslib;
}

BOOST_AUTO_TEST_CASE(DirectKinematics) {
    std::cout << "Test case DirectKinematics" << std::endl;
    using namespace mbslib;
    LBR2ModelAnalytic * modelAnalytic = new LBR2ModelAnalytic;
    LBR2ModelMbslib * modelMbslib = new LBR2ModelMbslib;
    LBR2ModelMbslib * modelMbslib2 = new LBR2ModelMbslib(LBR2ModelMbslib::Model::ModelDH2);
    modelMbslib->algorithm = LBR2ModelMbslib::CRBA;
    modelMbslib2->algorithm = LBR2ModelMbslib::CRBA;
    const size_t dof = 7;

    double mass = 0.0;
    mbslib::TVector3 rl;
    rl.setZero();

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    const MMSTs _q{-M_PI_2, M_PI_2, M_PI_2 / 5.0};

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<> _q_dis(_q.min, _q.max);

    mbslib::TVectorX q(dof);
    mbslib::TVectorX mbslibResult(3), analyticalResult(3);

    q.setZero();
    bool result;

    const size_t value_loops = 1e6,
                 total_loops = value_loops;

    for (size_t j = 0; j < value_loops; ++j) {
        for (size_t i = 0; i < dof; ++i) {
            q(i) = _q_dis(gen);
        }
        auto printResults = [&]() {
            std::cout << "q "
                      << q.transpose()
                      << std::endl;
            std::cout << "mbslib result: " << std::endl
                      << mbslibResult.transpose().format(LongFormat) << std::endl;
            std::cout << analyticalResult.transpose().format(LongFormat) << std::endl;
        };

        analyticalResult = modelAnalytic->forwardKinematics(q);

        mbslibResult = modelMbslib->forwardKinematics(q);
        TEST_CHECK_EQUAL_RESULT(analyticalResult, mbslibResult, result);

        if (!result)
            printResults();

        mbslibResult = modelMbslib2->forwardKinematics(q);
        TEST_CHECK_EQUAL_RESULT(analyticalResult, mbslibResult, result);

        if (!result)
            printResults();
    }
}

BOOST_AUTO_TEST_CASE(CoriolisTorque) {
    std::cout << "Test case CoriolisTorque" << std::endl;
    using namespace mbslib;
    LBR2ModelAnalytic * modelAnalytic = new LBR2ModelAnalytic;
    LBR2ModelMbslib * modelMbslib = new LBR2ModelMbslib;
    LBR2ModelMbslib * modelMbslib2 = new LBR2ModelMbslib(LBR2ModelMbslib::Model::ModelDH2);
    modelMbslib->algorithm = LBR2ModelMbslib::CRBA;
    modelMbslib2->algorithm = LBR2ModelMbslib::CRBA;
    const size_t dof = 7;

    double mass = 0.0;
    mbslib::TVector3 rl;
    rl.setZero();

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    const MMSTs _q{-M_PI_2, M_PI_2, M_PI_2 / 5.0};
    const MMSTs _dq{-M_PI_2, M_PI_2, M_PI_2 / 5.0};

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<> _q_dis(_q.min, _q.max);
    std::uniform_real_distribution<> _dq_dis(_dq.min, _dq.max);

    mbslib::TVectorX q(dof), dq(dof), ddq(dof);
    mbslib::TVectorX mbslibResult(dof), analyticalResult(dof);

    q.setZero();
    dq.setZero();
    ddq.setZero();
    bool result;

    const size_t value_loops = 1e6,
                 total_loops = value_loops;

    for (size_t j = 0; j < value_loops; ++j) {
        for (size_t i = 0; i < dof; ++i) {
            q(i) = _q_dis(gen);
            dq(i) = _dq_dis(gen);
        }

        auto printResults = [&]() {
            std::cout << "q "
                      << q.transpose()
                      << " dq "
                      << dq.transpose()
                      << std::endl;
            std::cout << "mbslib result: " << std::endl
                      << mbslibResult.transpose().format(LongFormat) << std::endl;
            std::cout << analyticalResult.transpose().format(LongFormat) << std::endl;
        };

        analyticalResult = modelAnalytic->inverseDynamics(q, dq, ddq);

        mbslibResult = modelMbslib->inverseDynamics(q, dq, ddq);

        TEST_CHECK_EQUAL_RESULT(analyticalResult, mbslibResult, result);

        if (!result)
            printResults();

        mbslibResult = modelMbslib2->inverseDynamics(q, dq, ddq);

        TEST_CHECK_EQUAL_RESULT(analyticalResult, mbslibResult, result);

        if (!result)
            printResults();
    }

    delete modelAnalytic;
    delete modelMbslib;
}

BOOST_AUTO_TEST_CASE(CalculateInvDyn) {
    std::cout << "Test case CalculateInvDyn" << std::endl;
    using namespace mbslib;
    LBR2ModelAnalytic * modelAnalytic = new LBR2ModelAnalytic;
    LBR2ModelMbslib * modelMbslib = new LBR2ModelMbslib;
    LBR2ModelMbslib * modelMbslib2 = new LBR2ModelMbslib(LBR2ModelMbslib::Model::ModelDH2);
    modelMbslib->algorithm = LBR2ModelMbslib::CRBA;
    modelMbslib2->algorithm = LBR2ModelMbslib::CRBA;
    const size_t dof = 7;

    double mass = 0.0;
    mbslib::TVector3 rl;
    rl.setZero();

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    const MMSTs _q{-M_PI_2, M_PI_2, M_PI_2 / 5.0};
    const MMSTs _dq{-M_PI_2, M_PI_2, M_PI_2 / 5.0};
    const MMSTs _ddq{-M_PI_2, M_PI_2, M_PI_2 / 5.0};

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<> _q_dis(_q.min, _q.max);
    std::uniform_real_distribution<> _dq_dis(_dq.min, _dq.max);
    std::uniform_real_distribution<> _ddq_dis(_ddq.min, _ddq.max);

    mbslib::TVectorX q(dof), dq(dof), ddq(dof);
    mbslib::TVectorX mbslibResult(dof), analyticalResult(dof);

    q.setZero();
    dq.setZero();
    ddq.setZero();
    bool result;

    const size_t value_loops = 1e6,
                 total_loops = value_loops;

    for (size_t j = 0; j < value_loops; ++j) {
        for (size_t i = 0; i < dof; ++i) {
            q(i) = _q_dis(gen);
            dq(i) = _dq_dis(gen);
            ddq(i) = _ddq_dis(gen);
        }

        auto printResults = [&]() {
            std::cout << "q "
                      << q.transpose()
                      << " dq "
                      << dq.transpose()
                      << " ddq "
                      << ddq.transpose()
                      << std::endl;
            std::cout << "mbslib result: " << std::endl
                      << mbslibResult.transpose().format(LongFormat) << std::endl;
            std::cout << analyticalResult.transpose().format(LongFormat) << std::endl;
        };

        analyticalResult = modelAnalytic->inverseDynamics(q, dq, ddq);

        mbslibResult = modelMbslib->inverseDynamics(q, dq, ddq);

        TEST_CHECK_EQUAL_RESULT(analyticalResult, mbslibResult, result);

        if (!result)
            printResults();

        mbslibResult = modelMbslib2->inverseDynamics(q, dq, ddq);

        TEST_CHECK_EQUAL_RESULT(analyticalResult, mbslibResult, result);

        if (!result)
            printResults();
    }

    delete modelAnalytic;
    delete modelMbslib;
}

BOOST_AUTO_TEST_CASE(CalculateDirDyn) {
    using namespace mbslib;
    LBR2ModelAnalytic * modelAnalytic = new LBR2ModelAnalytic;
    LBR2ModelMbslib * modelMbslib = new LBR2ModelMbslib;
    LBR2ModelMbslib * modelMbslib2 = new LBR2ModelMbslib(LBR2ModelMbslib::Model::ModelDH2);
    modelMbslib->algorithm = LBR2ModelMbslib::ABA;
    modelMbslib2->algorithm = LBR2ModelMbslib::CRBA;
    const size_t dof = 7;

    double mass = 0.0;
    mbslib::TVector3 rl;
    rl.setZero();

    Eigen::IOFormat LongFormat(20);
    std::cout << std::setprecision(40);

    const MMSTs _q{-M_PI_2, M_PI_2, M_PI_2 / 5.0};
    const MMSTs _dq{-M_PI_2, M_PI_2, M_PI_2 / 5.0};
    const MMSTs _tau{-10.0, 10.0, 5.0};

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<> _q_dis(_q.min, _q.max);
    std::uniform_real_distribution<> _dq_dis(_dq.min, _dq.max);
    std::uniform_real_distribution<> _tau_dis(_tau.min, _tau.max);

    mbslib::TVectorX q(dof), dq(dof), tau(dof);
    mbslib::TVectorX mbslibResult(dof), analyticalResult(dof), mbslibResult2(dof);
    mbslib::TVectorX mbslibResult3(dof), mbslibResult4(dof);

    bool result;

    const size_t value_loops = 1e6,
                 total_loops = value_loops;

    for (size_t j = 0; j < value_loops; ++j) {
        for (size_t i = 0; i < dof; ++i) {
            q(i) = _q_dis(gen);
            dq(i) = _dq_dis(gen);
            tau(i) = _tau_dis(gen);
        }

        auto printResults = [&]() {
            std::cout << "Iteration: " << (j + 1) << std::endl;
            std::cout << "q "
                      << q.transpose() << std::endl
                      << " dq "
                      << dq.transpose() << std::endl
                      << " tau "
                      << tau.transpose() << std::endl
                      << std::endl;
            std::cout << "mbslib result: "
                      << std::endl
                      << mbslibResult.transpose().format(LongFormat) << std::endl
                      << mbslibResult2.transpose().format(LongFormat) << std::endl;
            std::cout << analyticalResult.transpose().format(LongFormat) << std::endl;

            std::cout << "Mass Matrix: " << std::endl
                      << modelMbslib->massMatrix(q) << std::endl
                      << modelAnalytic->massMatrix(q) << std::endl;
        };

        analyticalResult = modelAnalytic->forwardDynamics(q, dq, tau);

        modelMbslib->algorithm = LBR2ModelMbslib::ABA;
        mbslibResult = modelMbslib->forwardDynamics(q, dq, tau);
        modelMbslib->algorithm = LBR2ModelMbslib::CRBA;
        mbslibResult2 = modelMbslib->forwardDynamics(q, dq, tau);

        TEST_CHECK_EQUAL_RESULT(analyticalResult, mbslibResult, result);
        TEST_CHECK_EQUAL_RESULT(analyticalResult, mbslibResult2, result);

        if (!result)
            printResults();

        modelMbslib2->algorithm = LBR2ModelMbslib::ABA;
        mbslibResult = modelMbslib2->forwardDynamics(q, dq, tau);
        modelMbslib2->algorithm = LBR2ModelMbslib::CRBA;
        mbslibResult2 = modelMbslib2->forwardDynamics(q, dq, tau);

        TEST_CHECK_EQUAL_RESULT(analyticalResult, mbslibResult, result);
        TEST_CHECK_EQUAL_RESULT(analyticalResult, mbslibResult2, result);

        if (!result)
            printResults();
    }

    delete modelAnalytic;
    delete modelMbslib;
}
