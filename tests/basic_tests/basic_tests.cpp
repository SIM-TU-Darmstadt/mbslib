#define BOOST_TEST_MODULE BasicTests
#include <boost/test/included/unit_test.hpp>

#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>
#include <mbslib/elements/joint/JointForceSetter.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>

using namespace mbslib;

BOOST_AUTO_TEST_CASE(BasicTest)
{

    TVectorX l(2);
    l << 1.0, 2.0;
    TVectorX m(2);
    m << 0.2, 0.1;

    TVectorX testValue(8);

    TScalar g = -9.81;

    TVector3Vector r_com;
    r_com.assign(2, TVector3::Zero());

    TMatrix3x3Vector I_com;
    I_com.assign(2, TMatrix3x3::Zero());

    TVectorX zero = TVectorX::Zero(2);

    MbsCompoundWithBuilder* mbs;
    Endpoint * ep1, *ep2;
    Joint1DOF *j1, *j2;
    RigidLink *rl1, *rl2;
    Fork *fork;
    FixedTranslation *ft1, *ft2;
    Base* base;
    //JointForceSetter *jfs1, *jfs2;

    TVector3 mbslibResult;

    mbs = new MbsCompoundWithBuilder();

    base = mbs->addFixedBase("base");

    j1 = mbs->addRevoluteJoint(TVector3(0, 0, 1), "q1");
    rl1 = mbs->addRigidLink(TVector3(0,l(0),0),r_com[0], m(0), I_com[0],"link1");
    j2 = mbs->addPrismaticJoint(TVector3(0, 1, 0), "q2");
    rl2 = mbs->addRigidLink(TVector3(0,0,l(1)),r_com[1], m(1), I_com[1],"link2");
    fork = mbs->addFork("fork");
        ft1 = mbs->addFixedTranslation(TVector3(0, 0, 0.5), "link3");
        ep1 = mbs->addEndpoint("ep1");
    ft2 = mbs->addFixedTranslation(TVector3(-0.5, 0, 0.0), "link4");
    ep2 = mbs->addEndpoint("ep2");

//    jfs1 = new JointForceSetter(*j1);
//    jfs2 = new JointForceSetter(*j2);
//    mbs->addForceGenerator(jfs1);
//    mbs->addForceGenerator(jfs2);

    /// Gravity test
    testValue.setRandom();
    mbs->setGravitation(testValue.segment(0,3));
    BOOST_CHECK_EQUAL(testValue.segment(0,3), mbs->getGravitation());

    /// Elements size tests
    BOOST_CHECK_EQUAL(10, mbs->getElements().size());

    /// Endpoint tests
    BOOST_CHECK_NE(ep1, ep2);

    BOOST_CHECK_EQUAL(ep2, &mbs->getEnd());

    BOOST_CHECK_EQUAL(ep1, mbs->getEndpointByName("ep1"));
    BOOST_CHECK_EQUAL(ep2, mbs->getEndpointByName("ep2"));

    BOOST_CHECK_EQUAL(ep1, mbs->getElementByName("ep1"));
    BOOST_CHECK_EQUAL(ep2, mbs->getElementByName("ep2"));

    BOOST_CHECK_EQUAL(ep1, mbs->getEndpoints()[0]);
    BOOST_CHECK_EQUAL(ep2, mbs->getEndpoints()[1]);

    BOOST_CHECK_EQUAL(2, mbs->getEndpoints().size());

    /// Joint tests
    BOOST_CHECK_NE(j1, j2);

    BOOST_CHECK_EQUAL(j1, mbs->getJointByName("q1"));
    BOOST_CHECK_EQUAL(j2, mbs->getJointByName("q2"));

    BOOST_CHECK_EQUAL(j1, mbs->getElementByName("q1"));
    BOOST_CHECK_EQUAL(j2, mbs->getElementByName("q2"));

    BOOST_CHECK_EQUAL(j1, mbs->getJoints()[0]);
    BOOST_CHECK_EQUAL(j2, mbs->getJoints()[1]);

    BOOST_CHECK_EQUAL(2, mbs->getJoints().size());

    BOOST_CHECK_EQUAL(2, mbs->getDOF());

    /// RigidLink tests
    BOOST_CHECK_NE(rl1, rl2);

    BOOST_CHECK_EQUAL(rl1, mbs->getElementByName("link1"));
    BOOST_CHECK_EQUAL(rl2, mbs->getElementByName("link2"));

    /// FixedTranslation tests
    BOOST_CHECK_NE(ft1, ft2);

    BOOST_CHECK_EQUAL(ft1, mbs->getElementByName("link3"));
    BOOST_CHECK_EQUAL(ft2, mbs->getElementByName("link4"));

    /// Base tests
    BOOST_CHECK_EQUAL(base, &mbs->getBase());

    BOOST_CHECK_EQUAL(base, mbs->getElementByName("base"));

    /// Fork tests
    BOOST_CHECK_EQUAL(fork, mbs->getElementByName("fork"));

    /// State tests
    for(size_t i = 0; i < 100; ++i) {
        testValue.setRandom();

        mbs->setJointPosition(testValue.segment(0,2));
        BOOST_CHECK_EQUAL(testValue.segment(0,2), mbs->getJointPosition());

        mbs->setJointVelocity(testValue.segment(2,2));
        BOOST_CHECK_EQUAL(testValue.segment(2,2), mbs->getJointVelocity());

        mbs->setJointAcceleration(testValue.segment(4,2));
        BOOST_CHECK_EQUAL(testValue.segment(4,2), mbs->getJointAcceleration());

        mbs->setJointForceTorque(testValue.segment(6,2));
        BOOST_CHECK_EQUAL(testValue.segment(6,2), mbs->getJointForceTorque());
        //BOOST_CHECK_EQUAL(mbs->getControlValues(), mbs->getJointForceTorque());

        BOOST_CHECK_EQUAL(testValue(0), mbs->getState()(0));
        BOOST_CHECK_EQUAL(testValue(1), mbs->getState()(2));
        BOOST_CHECK_EQUAL(testValue(2), mbs->getState()(1));
        BOOST_CHECK_EQUAL(testValue(3), mbs->getState()(3));
        BOOST_CHECK_EQUAL(testValue(2), mbs->getDStateDt()(0));
        BOOST_CHECK_EQUAL(testValue(3), mbs->getDStateDt()(2));
        BOOST_CHECK_EQUAL(testValue(4), mbs->getDStateDt()(1));
        BOOST_CHECK_EQUAL(testValue(5), mbs->getDStateDt()(3));

        testValue.setRandom();
        mbs->setState(testValue.segment(0,4));
        BOOST_CHECK_EQUAL(testValue.segment(0,4), mbs->getState());
        BOOST_CHECK_EQUAL(testValue(0), mbs->getJointPosition()(0));
        BOOST_CHECK_EQUAL(testValue(2), mbs->getJointPosition()(1));
        BOOST_CHECK_EQUAL(testValue(1), mbs->getJointVelocity()(0));
        BOOST_CHECK_EQUAL(testValue(3), mbs->getJointVelocity()(1));
        BOOST_CHECK_EQUAL(mbs->getDStateDt()(0), mbs->getState()(1));
        BOOST_CHECK_EQUAL(mbs->getDStateDt()(2), mbs->getState()(3));

    }

    /// Coordinate frame tests
    mbs->setJointPosition(TVectorX::Zero(2));
    mbs->doDirkin();

    BOOST_CHECK_EQUAL(TVector3::Zero(), j1->getCoordinateFrame().r);
    BOOST_CHECK_EQUAL(TVector3::Zero(), base->getCoordinateFrame().r);
    BOOST_CHECK_EQUAL(TVector3(0,l(0),0), j2->getCoordinateFrame().r);
    BOOST_CHECK_EQUAL(TVector3(0,l(0),0), rl1->getCoordinateFrame().r);
    BOOST_CHECK_EQUAL(TVector3(0,l(0),l(1)), rl2->getCoordinateFrame().r);
    BOOST_CHECK_EQUAL(TVector3(0,l(0),l(1)), fork->getCoordinateFrame().r);
    BOOST_CHECK_EQUAL(TVector3(0,l(0),l(1)+0.5), ft1->getCoordinateFrame().r);
    BOOST_CHECK_EQUAL(TVector3(0,l(0),l(1)+0.5), ep1->getCoordinateFrame().r);
    BOOST_CHECK_EQUAL(TVector3(-0.5,l(0),l(1)), ft2->getCoordinateFrame().r);
    BOOST_CHECK_EQUAL(TVector3(-0.5,l(0),l(1)), ep2->getCoordinateFrame().r);


    return;

    BOOST_CHECK_EQUAL (mbs->getJointPosition()(0), 10);

    mbs->setJointPosition(TVector3(20, 0, 2));
    BOOST_CHECK_EQUAL (mbs->getJointPosition()(0), 20);

    BOOST_CHECK_EQUAL (mbs->getGravitation()(0), 0);

    mbs->setGravitation(TVector3(0, 9.81, 0));
    BOOST_CHECK_EQUAL (mbs->getGravitation()(1), 9.81);

    BOOST_CHECK_EQUAL (mbs->getJointAcceleration()(0), 0);
}
