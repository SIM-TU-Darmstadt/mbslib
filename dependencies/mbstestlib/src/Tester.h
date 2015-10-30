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
 * @file Tester.h
 * Declaration of the Tester (uses the adapter interface)
 */

#ifndef TESTER_H_
#define TESTER_H_

#include "Adapter.hpp"
#include <cmath>
#include <vector>

//TODO: refactor i/o
#include <iostream>
#include <fstream>
#include "ErrorLogger.h"

#include "parserTools.h"
#include <string>
#include <sstream>

/** \class Tester <Tester.h>
 * Tester is the main component which contains the test-logic
 */
class Tester{
public:
  /**
   * Default Constructor
	 * 
	 * \params: 	testSpecListFileName : full path + name to a file which contains a list of test files
	 * 						adapter: the specific adapter which should be used
	 * 						logstream : (optionally) where to log to? (e.g. a file)
	 *          delta : (optionally) the global (absolute) tolerance
   */
  Tester(const std::string testSpecListFileName, Adapter * adapter, const std::string & testdir = "", const std::string & logfileName = "", TesterScalar deltaglobal = -1);

  /**
   * Destructor
   */
  ~Tester();

  /**
	 * Run the actual tests as specified in the test-file
	 */
	void doTests();

private:
	/**
	 * Guard to verify the Tester has all the infos it needs (i.e. a working testSpecFile-Stream, and an adapter)
	 */ 
  bool isValid;

	/**
	 * The file which contains the a list of files with test specifications
	 */
	std::ifstream testSpecListFile;

	/**
	 * The file which contains the test specifications (model + tests on this model)
	 */
	std::ifstream testSpecFile;

	/**
	 * Name of the file currently being read for testspecs
	 */
	std::string testSpecFileName;

	/**
	 * The file in which to write log and warning messages (if specified)
	 */
	std::ofstream logfile;

	/**
	 * The adapter to the framework/lib to run the test on (holds the model).
	 */
	Adapter * adapter;

	/**
	 * Indicates whether there were errors during the test run (default is false).
	 */
	bool testFailed;

  //--------------------------------------------------
  // testset vars
  //--------------------------------------------------

  std::string testdir;

	//TODO: comments
  unsigned int numJoints;

	// stores the absolute tolerance (used when comparing desired and actual values) (values <0 count as not set)
	TesterScalar delta; // testcase buffer (used for actual comparison)
	TesterScalar deltafile; // testlist buffer
	TesterScalar deltaglobal; // cmdline buffer

	TesterVector3 base_r;
	TesterMatrix3x3 base_R;
	TesterVector3 base_v;
	TesterVector3 base_omega;
	TesterVector3 base_vdot;
	TesterVector3 base_omegadot;

	TesterVector3 gravitiy;

	std::vector<TesterScalar> joints_q;
	std::vector<TesterScalar> joints_qdot;
	std::vector<TesterScalar> joints_qdotdot;
	std::vector<TesterScalar> joints_tau;

	TesterVector3 tcp_r;
	TesterMatrix3x3 tcp_R;
	TesterVector3 tcp_v;
	TesterVector3 tcp_omega;
	TesterVector3 tcp_vdot;
	TesterVector3 tcp_omegadot;

	TesterVector3 f_ext; // external force on TCP
	TesterVector3 n_ext; // external torque on TCP

  //--------------------------------------------------
  // helpers for doTests()
  //--------------------------------------------------

	//TODO: comments
	void readTestSet();
	/**
	 * reset all dynamic to zero
	 */
	void resetModel();

	/**
	 * prepare model (set props needed for specific problem)
	 */
	void prepDirKin();
	void prepInvDyn();
	void prepDirDyn();
	void prepHybDyn(const std::vector<bool> & directJointList);

	/**
	 * compare helpers :
	 * they check whether the difference is smaller than delta
	 * and set a test-failed flag if not as well as return a specifics to the error
	 */
	std::string compare(TesterScalar actual, TesterScalar desired);
	std::string compare(TesterVector3 actual, TesterVector3 desired);
	std::string compare(TesterMatrix3x3 actual, TesterMatrix3x3 desired);
	/**
	 * compare utility function (does the basic level comparison):
	 * @return: true if actual value does not exceed tolerance (i.e. relative error <= delta)
	 *          false else if actual value exceeds tolerance
	 *          BEWARE: if desired value =0 (+-10e-8) relative error has no meaning -
	 *                  an absolute value of delta will be used instead!
	 */
	bool withinTolerance(TesterScalar actual, TesterScalar desired);

	/**
	 * check TCP props (compared to the ones given in the test)
	 */
	void checkDirKin(std::string algorithmName);
	void checkInvDyn(std::string algorithmName);
	void checkDirDyn(std::string algorithmName);
	std::string checkHybDyn(const std::vector<bool> & directJointList);

	/************************************************************************/
	/* Helper to conveniently write int to strings (i.e. replace string)    */
	/************************************************************************/
	std::string iToStr(int input, std::string& output){
		std::stringstream stream;
		stream << input;
		output.clear();
		stream >> output;
		return output;
	}
	
}; // class Tester

// readability and code redundancy avoidance helper for checks
#define COMPARE(ACTUAL,DESIRED,ERRORPREFIX) 	compareResult = compare(ACTUAL, DESIRED); if (compareResult != ""){errorsDescription += ERRORPREFIX; errorsDescription += " "; errorsDescription += compareResult + "; \n";};

  /**
   * Debug value output generator
   */
#define GEN_DEBUG_MODEL_VALUES \
if (testFailed) \
{\
	std::stringstream joints_q_str;\
	std::stringstream joints_qdot_str;\
	std::stringstream joints_qdotdot_str;\
	std::stringstream joints_tau_str;\
	for (unsigned int i = 0; i < numJoints; i++){\
		joints_q_str << " " << adapter->getJointPos(i);\
		joints_qdot_str << " " << adapter->getJointVel(i);\
		joints_qdotdot_str << " " << adapter->getJointAcc(i);\
		joints_tau_str << " " << adapter->getJointForce(i);\
	}\
	\
	LOGOUT("       <debug method=\"" << algorithmName << "\">\n" << \
		"         <delta scalar=\"" << delta << "\"/>\n" <<\
		"         <base_r vector3=\"" << adapter->getBasePos() << "\"/>\n" <<\
		"         <base_R matrix3x3=\"" << adapter->getBaseOri() << "\"/>\n" <<\
		"         <base_v vector3=\"" << adapter->getBaseVel() << "\"/>\n" <<\
		"         <base_omega vector3=\"" << adapter->getBaseAngVel() << "\"/>\n" <<\
		"         <base_vdot vector3=\"" << adapter->getBaseAcc() << "\"/>\n" <<\
		"         <base_omegadot vector3=\"" << adapter->getBaseAngAcc() << "\"/>\n" <<\
		"         <gravitiy vector3=\"" << gravitiy << "\"/>\n" <<\
		"         <joints_q vector_n=\""<< joints_q_str.str() << "\"/>\n" <<\
		"         <joints_qdot vector_n=\"" << joints_qdot_str.str() << "\"/>\n" <<\
		"         <joints_qdotdot vector_n=\"" << joints_qdotdot_str.str() << "\"/>\n" <<\
		"         <joints_tau vector_n=\"" << joints_tau_str.str() << "\"/>\n" <<\
		"         <tcp_r vector3=\"" << adapter->getTCPPos() << "\"/>\n" <<\
		"         <tcp_R matrix3x3=\"" << adapter->getTCPOri() << "\"/>\n" <<\
		"         <tcp_v vector3=\"" << adapter->getTCPVel() << "\"/>\n" <<\
		"         <tcp_omega vector3=\"" << adapter->getTCPAngVel() <<"\"/>\n" <<\
		"         <tcp_vdot vector3=\"" << adapter->getTCPAcc() << "\"/>\n" <<\
		"         <tcp_omegadot vector3=\"" << adapter->getTCPAngAcc() << "\"/>\n" <<\
		"         <f_ext vector3=\"" << f_ext << "\"/>\n" <<\
		"         <n_ext vector3=\"" << n_ext << "\"/>\n" <<\
		"       </debug>");\
};

#endif /* TESTER_H_ */
