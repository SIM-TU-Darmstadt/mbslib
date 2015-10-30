/*
 * Copyright (C) 2009, 2010
 * Simulation, Systems Optimization and Robotics Group (SIM)
 * Technische Universitaet Darmstadt
 * Hochschulstr. 10
 * 64289 Darmstadt, Germany
 * www.sim.tu-darmstadt.de
 *
 * This file is part of the Multi-Robot-Simulation-Framework (MuRoSimF), www.murosimf.de.
 * All rights are reserved by the copyright holder.
 *
 * MuRoSimF is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You may use the MuRoSimF or parts of it only with the written permission of the copyright holder.
 * You may NOT modify or redistribute any part of MuRoSimF without the prior written
 * permission by the copyright holder.
 *
 * Any violation of the rights and restrictions mentioned above will be prosecuted by civil and penal law.
 * Any expenses associated with the prosecution will be charged against the violator.
 */

/**
 * @file Adapter.h
 * Declaration of the interface for test-adapter
 */

#ifndef ADAPTER_H_
#define ADAPTER_H_

#include "TesterTypes.h"
#include <string>
#include <vector>

/** \class Adapter <Adapter.h>
 * Adapter is an abstract interface-class for all test-adapters
 *
 * Each adapter is responsible for an adaption of the of the functions
 * which the tests require. I.e. it has to talk the individual framework
 * which is to be tested.
 *
 * Note: all units are in SI
 */
class Adapter{
public:
  /**
   * Default Constructor
   */
  Adapter(){}

  /**
   * Destructor
   */
  virtual ~Adapter() {}

	/**
	 * prepare the adapter for the use of a new model
	 * (i.e. the adapter has to be put in a state equivalent to a newly constructed one)
	 */
	virtual void clear()=0;

  //--------------------------------------------------
  // builder (joints must be remembered via (0 ... n-1) numbers )
  //--------------------------------------------------

  /**
   * add a base (fixed)
   *
   * \params:	r : position
   * 			R : orientation
   *
   */
  virtual void addFixedBase(const TesterVector3& r, const TesterMatrix3x3& R)=0;

	/**
   * add a base (free)
   *
   * \params:	r : position
   * 			R : orientation
   *
   */
  virtual void addFreeBase(const TesterVector3& r, const TesterMatrix3x3& R)=0;

  /**
   * add a fork
   *
   */
  virtual void addFork()=0;

  /**
   * add a fixed translation
   *
   * \params: 	r : positional offset
   * 			com: center off mass (convention: origin is end of the link)
   * 			m : mass
   * 			I : Inertia
   *
   */
  virtual void addTranslation(const TesterVector3& r, const TesterVector3& com, const TesterScalar m, const TesterMatrix3x3& I)=0;

  /**
   * add a fixed rotation (given a rotation matrix)
   *
   * \params:	R : the rotation matrix
   */
  virtual void addRotation(const TesterMatrix3x3& R)=0;

  /**
   * add a fixed rotation (given the rotation's axis and angle)
   *
   * \params:	a : the rotation axis (must be of length 1!)
   * 			alpha : the rotation angle (in rad)
   */
  virtual void addRotation(const TesterVector3& a, const TesterScalar alpha)=0;

  /**
   * add a variable translation
   *
   * \params:	d : the direction (the length will be used as an initial value for q)
   */
  virtual void addTranslationalJoint(const TesterVector3& d)=0;

  /**
   * add a variable rotation
   *
   * \params:	a : the rotation axis (must have length 1!
   */
  virtual void addRotationalJoint(const TesterVector3& a)=0;

  /**
   * close a branch (of a fork)
   */
	virtual void addEndPoint(const std::string& name="")=0;

  /**
   * finalize model creation (returns true on success)
   */
  virtual bool finalizeModel()=0;


  //--------------------------------------------------
  // getter+setter for joints (are assumed to be numbered 0 ... n-1)
  //--------------------------------------------------

  /**
   * set position q of joint
   */
  virtual void setJointPos(const TesterInt jointNumber, const TesterScalar value)=0;

  /**
   * get position q of joint
   */
  virtual TesterScalar getJointPos(const TesterInt jointNumber)=0;

  /**
   * set velocity qdot of joint
   */
  virtual void setJointVel(const TesterInt jointNumber, const TesterScalar value)=0;

  /**
   * get velocity qdot of joint
   */
  virtual TesterScalar getJointVel(const TesterInt jointNumber)=0;

  /**
   * set acceleration qdotdot of joint
   */
  virtual void setJointAcc(const TesterInt jointNumber, const TesterScalar value)=0;

  /**
   * get acceleration qdotdot of joint
   */
	virtual TesterScalar getJointAcc(const TesterInt jointNumber)=0;

  /**
   * set force tau of joint
   */
  virtual void setJointForce(const TesterInt jointNumber, const TesterScalar value)=0;

  /**
   * get force tau of joint
   */
  virtual TesterScalar getJointForce(const TesterInt jointNumber)=0;

  //--------------------------------------------------
  // getter+setter for TCP (Tool-Center-Point) (only one TCP in Model) (including external forces)
  //--------------------------------------------------

  /**
   * get position r of TCP
   */
  virtual TesterVector3 getTCPPos()=0;

  /**
   * get orientation R of TCP
   */
  virtual TesterMatrix3x3 getTCPOri()=0;

  /**
   * get linear velocity v of TCP
   */
  virtual TesterVector3 getTCPVel()=0;

  /**
   * get angular velocity omega of TCP
   */
  virtual TesterVector3 getTCPAngVel()=0;

  /**
   * get linear acceleration vdot of TCP
   */
  virtual TesterVector3 getTCPAcc()=0;

  /**
   * get angular acceleration omegadot of TCP
   */
  virtual TesterVector3 getTCPAngAcc()=0;

  /**
   * set external force and torque
   */
	virtual void setExtForceAndTorque(const TesterVector3 force, const TesterVector3 torque)=0;

  //--------------------------------------------------
  // getter+setter for base
  //--------------------------------------------------

	/**
   * get position of base
   */
	virtual TesterVector3 getBasePos()=0;

	/**
   * get orientation of base
   */
	virtual TesterMatrix3x3 getBaseOri()=0;

	/**
   * get velocity of base
   */
	virtual TesterVector3 getBaseVel()=0;

	/**
   * get angular velocity of base
   */
	virtual TesterVector3 getBaseAngVel()=0;

	/**
   * get acceleration of base
   */
	virtual TesterVector3 getBaseAcc()=0;

	/**
   * get angular acceleration of base
   */
	virtual TesterVector3 getBaseAngAcc()=0;


	/**
   * set position of base
   */
	virtual void setBasePos(const TesterVector3 value)=0;

	/**
   * set orientation of base
   */
	virtual void setBaseOri(const TesterMatrix3x3 value)=0;

	/**
   * set velocity of base
   */
	virtual void setBaseVel(const TesterVector3 value)=0;

	/**
   * set angular velocity of base
   */
	virtual void setBaseAngVel(const TesterVector3 value)=0;

	/**
   * set acceleration of base
   */
	virtual void setBaseAcc(const TesterVector3 value)=0;

	/**
   * set angular acceleration of base
   */
	virtual void setBaseAngAcc(const TesterVector3 value)=0;

  //--------------------------------------------------
  // setters for gravity
  //--------------------------------------------------

	/**
   * set gravitiy (adapter-note: this function will be called after the base properties have been set)
   */
	virtual void setGravitiy(const TesterVector3 value)=0;

  //--------------------------------------------------
  // algorithm calls
  //--------------------------------------------------

  /**
   * executes direct kinematics in the framework
	 *
	 * @return: the name of the algorithm which was executed
	 *              empty if the selected algorithm could not be executed
   *              (e.g. due to an unfitting robot model)
	 * \param algorithmNumber: which algorithm to execute
	 *
   */
  virtual std::string doDirKin(unsigned int algorithmNumber)=0;

  /**
   * how many direct kinematics algorithms are in the framework
	 * (keep this in sync with doDirKin() and the associated algo-caller-functions)
   */
	virtual unsigned int getNumberOfDirKinAlgorithms()=0;

  /**
   * executes inverse dynamics in the framework (q_dotdot -> tau)
	 *
	 * @return: the name of the algorithm which was executed
	 *              empty if the selected algorithm could not be executed
	 *              (e.g. due to an unfitting robot model)
	 * \param algorithmNumber: which algorithm to execute
	 *
   */
  virtual std::string doInvDyn(unsigned int algorithmNumber)=0;

  /**
   * how many inverse dynamics algorithms are in the framework
	 * (keep this in sync with doInvDyn() and the associated algo-caller-functions)
   */
	virtual unsigned int getNumberOfInvDynAlgorithms()=0;

  /**
   * executes forward dynamics in the framework (tau -> q_dotdot)
   *
	 * @return: the name of the algorithm which was executed
	 *              empty if the selected algorithm could not be executed
	 *              (e.g. due to an unfitting robot model)
	 * \param algorithmNumber: which algorithm to execute
	 *
   */
  virtual std::string doDirDyn(unsigned int algorithmNumber)=0;

  /**
   * how many forward dynamics algorithms are in the framework
	 * (keep this in sync with doDirDyn() and the associated algo-caller-functions)
   */
	virtual unsigned int getNumberOfDirDynAlgorithms()=0;


  /**
   * executes hybrid dynamics in the framework
   *
	 * @return: the name of the algorithm which was executed
	 *              empty if the selected algorithm could not be executed
	 *              (e.g. due to an unfitting robot model)
	 * \param algorithmNumber: which algorithm to execute
	 * \param directJointList: list of joints for which to calc direct dynamics
	 *                         (true = direct dynamics for this joint
	 *                          false = inverse dynamics for this joint)
	 *
   */
  virtual std::string doHybDyn(unsigned int algorithmNumber, const std::vector<bool> & directJointList)=0;

  /**
   * how many hybrid dynamics algorithms are in the framework
	 * (keep this in sync with doDirDyn() and the associated algo-caller-functions)
   */
	virtual unsigned int getNumberOfHybDynAlgorithms()=0;


}; // class Adapter



#endif /* ADAPTER_H_ */
