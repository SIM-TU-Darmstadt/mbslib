extern "C" {
#include <motionEquation.h>
}

#include <Eigen/Core>
#include <iostream>
#include <mbslib/elements/drive/PassiveSpringDamperDrive.hpp>
#include <mbslib/elements/spring/model/LinearSpringModel.hpp>
#include <mbslib/mbslib.hpp>

namespace mbslib {
typedef MbsObject * MbsObjectPtr;
typedef MbsCompoundWithBuilder * MbsCompoundWithBuilderPtr;
typedef Endpoint * EndpointPtr;
}

struct Data {
    static Data * data;
    static Data * getInstance() {
        if (data == nullptr) {
            data = new Data;
        }
        return data;
    }

#ifdef USE_ADOLC
    mbslib::DeriveOMat * dom;
#endif
    mbslib::MbsCompoundWithBuilder * mbs;
    mbslib::LinearSpringModel * springModel;
    mbslib::PassiveSpringDamperDrive * psdd;
    Data() {
#ifdef USE_ADOLC
        dom = nullptr;
#endif
        mbs = nullptr;
        springModel = nullptr;
        psdd = nullptr;
    }
    virtual ~Data() {
        delete mbs;
        mbs = nullptr;
        delete springModel;
        springModel = nullptr;
        delete psdd;
        psdd = nullptr;
#ifdef USE_ADOLC
        delete dom;
        dom = nullptr;
#endif
    }
    double l;
    double m;
    double k;
    double d;
    double g;
};

Data * Data::data = nullptr;

extern "C" {

//static Data* data = nullptr;

void lib_init(double l, double m, double k, double d, double g) {
    using namespace mbslib;

    Data * data = Data::getInstance();
    data->mbs = new mbslib::MbsCompoundWithBuilder("Bla");
    mbslib::MbsCompoundWithBuilder * mbs = data->mbs;

    mbs->addFixedBase();

    mbslib::TVector3 dir;
    dir << 1.0, 0.0, 0.0;
    mbs->addPrismaticJoint(dir, "q1");

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
    mbs->addEndpoint(m, I, "pulley_com");

    mbs->addEndpoint("pulley");

    dir << 0.0, g, 0.0;
    mbs->setGravitation(dir);

    data->psdd = new PassiveSpringDamperDrive(*q2, k, d, "PSDD");
    ((MbsCompound *)mbs)->addDrive(data->psdd);
    //data->springModel = new mbslib::LinearSpringModel(k, d);
    //mbs->addSpring(*data->springModel, *q2);

    data->l = l;
    data->m = m;
    data->k = k;
    data->d = d;
    data->g = g;

#ifdef USE_ADOLC
    data->dom = new mbslib::DeriveOMat(*data->mbs);

    data->dom->addIndependent("PSDD", "springconstant");
    data->dom->addDependent("q1", "ddq");
    data->dom->addDependent("q2", "ddq");

    mbslib::TVectorX q(2), qp(2), qpp(2), tau(2);
    q << 1.0, 1.0;
    qp << 1.0, 1.0;
    qpp << 1.0, 1.0;
    tau.setZero();
    data->mbs->setJointPosition(q);
    data->mbs->setJointVelocity(qp);
    data->mbs->setJointForceTorque(tau);
    data->mbs->setJointAcceleration(qpp);

#endif
}

static double sgn(const double v) {
    if (v > 0)
        return 1;
    if (v < 0)
        return -1;
    return 0;
}

void motionEquation(const double * x, double * xp) {
    Data * data = Data::getInstance();
    mbslib::TVectorX q(2), qp(2);
    q << x[0], x[2];
    qp << x[1], x[3];
    data->mbs->setJointPosition(q);
    data->mbs->setJointVelocity(qp);

    mbslib::TVectorX tau(2);
    tau.setZero();

    data->mbs->setJointForceTorque(tau);

    //data->mbs->doABA();
    data->mbs->doForwardDrives();
    data->mbs->doCrba();

    mbslib::TVectorX qpp = data->mbs->getJointAcceleration();

#ifdef USE_ADOLC
    xp[0] = x[1];
    xp[1] = qpp[0].value();
    xp[2] = x[3];
    xp[3] = qpp[1].value();
#else
    xp[0] = x[1];
    xp[1] = qpp[0];
    xp[2] = x[3];
    xp[3] = qpp[1];
#endif
}

void kinematics(const double * x, double * pos, double * vel) {
    Data * data = Data::getInstance();
    mbslib::TVectorX q(2), qp(2);
    q << x[0], x[2];
    qp << x[1], x[3];
    data->mbs->setJointPosition(q);
    data->mbs->setJointVelocity(qp);

    data->mbs->doDirkin();

    mbslib::TVector3 p1 = data->mbs->getJoints().back()->getCoordinateFrame().r;
    mbslib::TVector3 p2 = data->mbs->getEnd().getCoordinateFrame().r;

#ifdef USE_ADOLC
    pos[0] = p1(0).value();
    pos[1] = p1(1).value();
    pos[2] = p2(0).value();
    pos[3] = p2(1).value();
#else
    pos[0] = p1(0);
    pos[1] = p1(1);
    pos[2] = p2(0);
    pos[3] = p2(1);
#endif

    if (vel != nullptr) {
        mbslib::TVector3 v1 = data->mbs->getJoints().back()->getCoordinateFrame().R * data->mbs->getJoints().back()->getCoordinateFrame().v;
        mbslib::TVector3 v2 = data->mbs->getEnd().getCoordinateFrame().R * data->mbs->getEnd().getCoordinateFrame().v;
#ifdef USE_ADOLC
        vel[0] = v1(0).value();
        vel[1] = v1(1).value();
        vel[2] = v2(0).value();
        vel[3] = v2(1).value();
#else
        vel[0] = v1(0);
        vel[1] = v1(1);
        vel[2] = v2(0);
        vel[3] = v2(1);
#endif
    }
}

#ifdef USE_ADOLC
void derivatives(const double * x, double * results) {
    Data * data = Data::getInstance();
    mbslib::TVectorX q(2), qp(2), qpp(2), tau(2);
    q << x[0], x[2];
    qp << x[1], x[3];
    qpp.setZero();
    tau.setZero();
    data->mbs->setJointPosition(q);
    data->mbs->setJointVelocity(qp);
    data->mbs->setJointForceTorque(tau);
    data->mbs->setJointAcceleration(qpp);

    data->dom->startTape();
    data->mbs->doForwardDrives();
    data->mbs->doCrba();
    data->dom->endTape();

    //    mbslib::DomMatrix m_ = data->dom->calculateJacobian();
    //    mbslib::DomMatrix m; m.resize(1,1);
    //    m(0,0) = m_(0,0);
    //    mbslib::DomVector v1 = mbslib::convert(m);
    //    Eigen::Map<Eigen::VectorXd> v2(results, v1.size());
    //    v2 = v1;

    mbslib::DomMatrix m = data->dom->calculateJacobian();
    mbslib::DomVector v1 = mbslib::convert(m);
    Eigen::Map< Eigen::VectorXd > v2(results, v1.size());
    v2 = v1;

    /*    const double q2 = x[2];
    const double dq2 = x[3];
    results[0] = - (3.0 * cos(q2) * (d * dq2 + k * q2)) / (2.0 * m * m * l);*/
}
#endif

void lib_cleanup() {
    delete Data::getInstance();
    Data::data = nullptr;
}
}
