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
 * @file Tester.cpp
 * Definition of the Tester
 */

#include "Tester.h"
#include "RobotModelParser.h"


//using namespace mbslib;

Tester::Tester(const std::string testSpecListFileName, Adapter * adapter, const std::string & testdir, const std::string & logfileName, TesterScalar deltaglobal)
:testdir(testdir), deltaglobal(deltaglobal)
{
	isValid = true; // assume everything will work out (set isValid to false on failures)

	// set Error stream
  ErrorLogger::setErrorStream(std::cout); // set error output to console

	// open the file with the list of files with tests specifications
  std::string filename = (testdir + testSpecListFileName);
	testSpecListFile.open(filename.c_str());
	if (!testSpecListFile.good()) {
		isValid = false;
		ERROUT("File access error! (check the testlistfile parameter: " + filename + ")");
	}

	// store which adapter to use for the tests
	if (adapter == NULL) {
		isValid = false;
    ERROUT("Adapter-object access error! (check the class that calls Tester)");
	} else{
  	this->adapter = adapter;
	};

	// prepare logging
	std::ostream* logstream;
	// prepare external logfile (if logfile parameter specifies one)
	if (!logfileName.empty()){
		logfile.open(logfileName.c_str(), std::ios::trunc);
		if (!logfile.good()) {
			ERROUT("File access error! (unable to write logfile)");
		}
		logstream = &logfile;
	} else {
		logstream = &std::cout;
	}

	// set warning and logging streams
	ErrorLogger::setLogStream(*logstream); // set log output to console
	ErrorLogger::setWarnStream(*logstream); // set warning output to console
};

Tester::~Tester() 
{

};

void Tester::doTests(){
LOGOUT("<mbs_test_protocol>");
// loop through each testSpecFile specified in the testSpecListFile (i.e. parse model and do the tests for all datasets)
while(fileHasAdditionalContent(testSpecListFile)){
	// init delta for new testfile
	deltafile = -1;
	// init with empty line
	testSpecFileName = "";
	// if the current line was empty skip until a non empty line
	while (testSpecFileName.length() == 0) {
		// get next testSpecFile
		std::getline(testSpecListFile,testSpecFileName);
		// workaround for windows line endings under linux removing '\r'
		trim(testSpecFileName);
	}
	// open it
	testSpecFile.close();
	testSpecFile.clear();
  std::string filename = (testdir+testSpecFileName);
	//if first character of line is %, ignore the line
	if(testSpecFileName.at(0) == '%'){continue;}
	testSpecFile.open( filename.c_str());
	if (!testSpecFile.good()) {
		isValid = false;
		ERROUT("File access error! (check the the list of tests: " + filename + ")");
		break;
	}
	// parse delta (tolerance) value
	PARSE_DELTA(testSpecListFile,deltafile);
	//LOGOUT("  <mbs file=\"" << testSpecFileName << "\">"); // moved after model parsing to get model-string into attribute
		
	// clean up the model for next run
	adapter->clear();

	// parse the model
	RobotModelParser parser;
	int rv = parser.readRobotModel(testSpecFile,adapter);
	if (rv < 0){
		ERROUT("Failed to read model from file!");
		return; // an error occured during parsing - stop doing the tests on a faulty/empty model
	}else numJoints = rv;

	// check if we are all set
	if (!this->isValid) {
		ERROUT("Tester not valid error! (check the test-spec-file and the adapter - i.e. the constructor arguments)");
		return;
	}
	// start logging file + model
	LOGOUT("  <mbs file=\"" << testSpecFileName << "\" desc=\"" << parser.getModelAsString() << "\">");

	//
	// for each dataset in the testspecfile :
	//
	// read the properties q, qdot, qdotdot (set all else to 0)
	// doDirKin()
	// check TCP props (compared to the ones given in the test)
	//
  // read the properties q, qdot, qdotdot (set all else to 0)
	// doInvDyn()
	// check props + tau (compared to the ones given in the test)
	//
	// read the properties q, qdot, tau (set all else to 0)
	// doDirDyn()
	// check props + qdotdot (compared to the ones given in the test)
	//	
	// read the properties q, qdot, tau (set all else to 0) (for dirDyn joints)
	//                     q, qdot, qdotdot (set all else to 0) (for invDyn joints)
	// doHybridDyn() for all combinations
	// check props + qdotdot/tau (compared to the ones given in the test) (qdotdot for a dirDyn joint, tau for an invDyn joint)
	//
	int testSetCounter = 0;
	while(fileHasAdditionalContent(testSpecFile)){
		// parse description
		std::string datasetDescription = "";
		while (datasetDescription == ""){ // skip empty lines
		  std::getline(testSpecFile, datasetDescription); // read description
		  trim(datasetDescription);
		}
		LOGOUT("    <case nr=\"" << ++testSetCounter << "\"" << " desc=\"" << datasetDescription << "\">");
		//LOGOUT("Processing data set number " << ++testSetCounter << " of file " << testSpecFileName << " . . .");
		//LOGOUT("Description: " << datasetDescription << "\n");

		// read testset values from file
		readTestSet();

		// direct kinematics (all available algorithms)
		for (unsigned int i = 0; i < adapter->getNumberOfDirKinAlgorithms(); i++){
		  prepDirKin();
		  checkDirKin(adapter->doDirKin(i));
		};

		// inverse dynamics (all available algorithms)
	  for (unsigned int i = 0; i < adapter->getNumberOfInvDynAlgorithms(); i++){
		  prepInvDyn();
		  checkInvDyn(adapter->doInvDyn(i));
		};

		// forward dynamics (all available algorithms)
	  for (unsigned int i = 0; i < adapter->getNumberOfDirDynAlgorithms(); i++){
		  prepDirDyn();
		  checkDirDyn(adapter->doDirDyn(i));
		};

		// hybrid dynamics (all available algorithms)
		for (unsigned int i = 0; i < adapter->getNumberOfHybDynAlgorithms(); i++){
			std::vector<bool> directJointList;
			directJointList.resize(numJoints, false); // initialize jointList with InvDyn for all joints
			std::string errorsDescriptionBuffer; // error desc buffer (for current algorithm only - but all hybrid cases)
			std::string algorithmName; // name of the current hyb dyn method

			// run test for all (2^n) combinations of directJointList
			//assert(numJoints < ); //TODO: add check for numJoints < log_2(sizeop(unsiged int)) and rework static_cast
			for (unsigned int j = 0; j < static_cast<unsigned int>((1 << numJoints)); j++){
				// convert value of indexed variable to binary representation in directJointList
				unsigned int indexedVarValue = j;
				for (unsigned int k = 0; k < numJoints; k++) {
					directJointList.at(k) = (indexedVarValue % 2)!=0;
					indexedVarValue /= 2;
				}

			  prepHybDyn(directJointList);
			  algorithmName = adapter->doHybDyn(i, directJointList); // do hyb dyn and remember which method
			  if (algorithmName == "") break; // if the algorithm was not run skip the checks (also no log outputs in this case) (and skip all other hybrid combinations)
				errorsDescriptionBuffer += checkHybDyn(directJointList);
			}

			// log checks
			if (errorsDescriptionBuffer.length() > 0) LOGOUT("      <hybdyn method=\"" << algorithmName << "\" ok=\"0\">" << errorsDescriptionBuffer << "</hybdyn>")
			else LOGOUT("      <hybdyn method=\"" << algorithmName << "\" ok=\"1\"/>");
		};

		LOGOUT("    </case>");
	}
  LOGOUT("  </mbs>");
}
LOGOUT("</mbs_test_protocol>");
};


void Tester::readTestSet(){
	// assumption: we are right before the next testset in input-stream
	
	//TODO: also parse units

	PARSE_DELTA(testSpecFile,delta);
	// select the delta value with the highest priority (global > testlistfile > testset)
	if (deltaglobal >= 0) delta = deltaglobal;
	else if (deltafile >= 0) delta = deltafile;
	else if (delta < 0) ERROUT("At least one testcase in \"" << testSpecFileName << "\" is missing a delta (is set to -1).")

	//LOGOUT(testSpecFile.rdstate());
	//LOGOUT(testSpecFile.fail());
	//testSpecFile.clear();
	//LOGOUT(testSpecFile.rdstate());
	//std::string trash;
	//testSpecFile >> trash;

	READVECTOR(base_r);
	READMATRIX(base_R);
	READVECTOR(base_v);
	READVECTOR(base_omega);
	READVECTOR(base_vdot);
	READVECTOR(base_omegadot);

	READVECTOR(gravitiy);

	READARRAY(joints_q,numJoints);
	READARRAY(joints_qdot,numJoints);
	READARRAY(joints_qdotdot,numJoints);
	READARRAY(joints_tau,numJoints);

	READVECTOR(tcp_r);
	READMATRIX(tcp_R);
	READVECTOR(tcp_v);
	READVECTOR(tcp_omega);
	READVECTOR(tcp_vdot);
	READVECTOR(tcp_omegadot);

	READVECTOR(f_ext);
	READVECTOR(n_ext);

};

void Tester::resetModel(){

	//adapter
	adapter->setBasePos(base_r);
	adapter->setBaseOri(base_R);
	adapter->setBaseVel(base_v);
 	adapter->setBaseAngVel(base_omega);
	adapter->setBaseAcc(base_vdot);
 	adapter->setBaseAngAcc(base_omegadot);

	adapter->setGravitiy(gravitiy);

	// set all joint parameters to zero (so we can set the appropriate ones in the tests)
	for (unsigned int i = 0; i < numJoints; i++){
		adapter->setJointPos(i, TesterScalar());
		adapter->setJointVel(i, TesterScalar());
		adapter->setJointAcc(i, TesterScalar());
		adapter->setJointForce(i, TesterScalar());
	};

	adapter->setExtForceAndTorque(f_ext,n_ext);
};


void Tester::prepDirKin(){
	resetModel();

	testFailed = false;
	
	// set q, q_dot and q_dotdot (position, velocity and acceleration of the joints)
	for (unsigned int i = 0; i < numJoints; i++){
		adapter->setJointPos(i, joints_q.at(i));
		adapter->setJointVel(i, joints_qdot.at(i));
		adapter->setJointAcc(i, joints_qdotdot.at(i));
	};

};

void Tester::prepInvDyn(){
	prepDirKin();
};

void Tester::prepDirDyn(){
	resetModel();

	testFailed = false;

	// set q, q_dot and tau (position, velocity and force/torque of the joints)
	for (unsigned int i = 0; i < numJoints; i++){
		adapter->setJointPos(i, joints_q.at(i));
		adapter->setJointVel(i, joints_qdot.at(i));
		adapter->setJointForce(i, joints_tau.at(i));
	};
};

void Tester::prepHybDyn(const std::vector<bool> & directJointList){
	resetModel();

	testFailed = false;

	// set q, q_dot, q_dotdot and tau  (position, velocity, acceleration and force of the joints)
	for (unsigned int i = 0; i < numJoints; i++){
		adapter->setJointPos(i, joints_q.at(i));
		adapter->setJointVel(i, joints_qdot.at(i));
		if (directJointList.at(i) == true) {
			// for direct dyn set acc=0 and tau=given value
			adapter->setJointAcc(i, TesterScalar());
			adapter->setJointForce(i, joints_tau.at(i));
		} else {
			// for inv dyn set acc=given and tau=0 value
			adapter->setJointAcc(i, joints_qdotdot.at(i));
			adapter->setJointForce(i, TesterScalar());
		}
	};

};

bool Tester::withinTolerance(TesterScalar actual, TesterScalar desired){
	// absolute errors
	//return std::abs(desired - actual) < delta;

	// relative errors (desired considered to be tolerance factor (0.01 means 1 %)
	// if desired =0 the relative error has no meaning - use absolute delta instead
	const TesterScalar zeroThreshold = 0.00000001;
	if ((desired < zeroThreshold) || (desired > -zeroThreshold)) {
		return std::abs(desired - actual) <= delta;
	} else {
		return std::abs(desired - actual) <= std::abs(delta * desired);
	}
};


std::string Tester::compare(TesterScalar actual, TesterScalar desired){
	if ( ! withinTolerance(desired, actual) ) {
		testFailed = true;
		std::stringstream returnBuffer;
		returnBuffer << "Supposed to be: " << desired << ", but is: " << actual;
		return returnBuffer.str();
	} else return "";
};

std::string Tester::compare(TesterVector3 actual, TesterVector3 desired){
	if ( ! (
		      withinTolerance(desired.vectorValues[0], actual.vectorValues[0])
					&&
					withinTolerance(desired.vectorValues[1], actual.vectorValues[1])
					&&
					withinTolerance(desired.vectorValues[2], actual.vectorValues[2])
					) ) {
		testFailed = true;
		std::stringstream returnBuffer;
		returnBuffer << "Supposed to be: " << desired.vectorValues[0] << " " << desired.vectorValues[1] << " " << desired.vectorValues[2]	
								<< ", but is: " <<	actual.vectorValues[0] << " " << actual.vectorValues[1] << " " << actual.vectorValues[2];
		return returnBuffer.str();
	} else return "";
};

std::string Tester::compare(TesterMatrix3x3 actual, TesterMatrix3x3 desired){
	bool comparefailed = false;
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++) {
			if (!( withinTolerance(desired.matrixValues[i][j], actual.matrixValues[i][j]))) {
				comparefailed = true;
			};
		};
	};
	if (comparefailed) {
  	testFailed = true;
		std::stringstream returnBuffer;
		returnBuffer << "Supposed to be: "
			<< desired.matrixValues[0][0] << " " << desired.matrixValues[0][1] << " " << desired.matrixValues[0][2] << "  "
			<< desired.matrixValues[1][0] << " " << desired.matrixValues[1][1] << " " << desired.matrixValues[1][2] << "  "
			<< desired.matrixValues[2][0] << " " << desired.matrixValues[2][1] << " " << desired.matrixValues[2][2] << "  "			
			<< ", but is: "
			<< actual.matrixValues[0][0] << " " << actual.matrixValues[0][1] << " " << actual.matrixValues[0][2] << "  "
			<< actual.matrixValues[1][0] << " " << actual.matrixValues[1][1] << " " << actual.matrixValues[1][2] << "  "
			<< actual.matrixValues[2][0] << " " << actual.matrixValues[2][1] << " " << actual.matrixValues[2][2];
		return returnBuffer.str();
	} else return "";
};

void Tester::checkDirKin(std::string algorithmName){
	if (algorithmName == "") return; // if the algorithm was not run skip the checks (also no log outputs in this case)
	std::string errorsDescription = "";
	std::string compareResult = "";
	std::string jointNumberBuffer = "";

	// check whether set props were left untouched
	// TODO: change text in case of a free base (then the problem is not the change but the value)
	// TODO: check whether these check produce false errors -> dirKin does not really set base values?!
	COMPARE(adapter->getBasePos(), base_r, "Base position was changed.")
  COMPARE(adapter->getBaseOri(), base_R, "Base orientation was changed.")
	COMPARE(adapter->getBaseVel(), base_v, "Base velocity was changed.");
 	COMPARE(adapter->getBaseAngVel(), base_omega, "Base angular velocity was changed.");
 	COMPARE(adapter->getBaseAcc(), base_vdot, "Base acceleration was changed.");
 	COMPARE(adapter->getBaseAngAcc(), base_omegadot, "Base angular acceleration was changed.");

	//COMPARE(adapter->getGravitiy(), gravitiy, "Gravity was changed.");

	for (unsigned int i = 0; i < numJoints; i++){
		iToStr(i, jointNumberBuffer);
		COMPARE(adapter->getJointPos(i), joints_q.at(i), "Joint " + jointNumberBuffer + " position was changed.");
		COMPARE(adapter->getJointVel(i), joints_qdot.at(i), "Joint " + jointNumberBuffer + "velocity was changed.");
		COMPARE(adapter->getJointAcc(i), joints_qdotdot.at(i), "Joint " + jointNumberBuffer + "acceleration was changed.");
		COMPARE(adapter->getJointForce(i), TesterScalar(), "Joint " + jointNumberBuffer + "force was changed.");
	};

	//COMPARE(adapter->getExtForce(), f_ext, "External force was changed.");
	//COMPARE(adapter->getExtTorque(), n_ext, "External torque was changed.");


	// check TCP props
	COMPARE(adapter->getTCPPos(), tcp_r, "TCP position wrong.");
	COMPARE(adapter->getTCPOri(), tcp_R, "TCP orientation wrong.");
	COMPARE(adapter->getTCPVel(), tcp_v, "TCP velocity wrong.");
	COMPARE(adapter->getTCPAngVel(), tcp_omega, "TCP angular velocity wrong.");
	// only check propagation of accelerations if no external forces are present
	// otherwise the direct kinematics test won't be able to verify this propagation (because it does not do dynamics)
	if (gravitiy.isZero() && f_ext.isZero() && n_ext.isZero()) {
 		COMPARE(adapter->getTCPAcc(), tcp_vdot, "TCP acceleration wrong.");
 		COMPARE(adapter->getTCPAngAcc(), tcp_omegadot, "TCP angular acceleration wrong.");
	}

	// give general feedback for the dirkin algo
	if (testFailed) LOGOUT("      <dirkin method=\"" << algorithmName << "\" ok=\"0\">" << errorsDescription << "</dirkin>")
	else LOGOUT("      <dirkin method=\"" << algorithmName << "\" ok=\"1\"/>");
	GEN_DEBUG_MODEL_VALUES
};

void Tester::checkInvDyn(std::string algorithmName){
	if (algorithmName == "") return; // if the algorithm was not run skip the checks (also no log outputs in this case)
	std::string errorsDescription = "";
	std::string compareResult = "";
	std::string jointNumberBuffer = "";

	// check whether set props were left untouched
	// TODO: change text in case of a free base (then the problem is not the change but the value)
	COMPARE(adapter->getBasePos(), base_r, "Base position was changed.")
	COMPARE(adapter->getBaseOri(), base_R, "Base orientation was changed.")
	COMPARE(adapter->getBaseVel(), base_v, "Base velocity was changed.");
	COMPARE(adapter->getBaseAngVel(), base_omega, "Base angular velocity was changed.");
	COMPARE(adapter->getBaseAcc(), base_vdot, "Base acceleration was changed.");
	COMPARE(adapter->getBaseAngAcc(), base_omegadot, "Base angular acceleration was changed.");

	//COMPARE(adapter->getGravitiy(), gravitiy, "Gravity was changed.");

	for (unsigned int i = 0; i < numJoints; i++){
		iToStr(i, jointNumberBuffer);
		COMPARE(adapter->getJointPos(i), joints_q.at(i), "Joint " + jointNumberBuffer + " position was changed.");
		COMPARE(adapter->getJointVel(i), joints_qdot.at(i), "Joint " + jointNumberBuffer + " velocity was changed.");
		COMPARE(adapter->getJointAcc(i), joints_qdotdot.at(i), "Joint " + jointNumberBuffer + " acceleration was changed.");
	};

	//COMPARE(adapter->getExtForce(), f_ext, "External force was changed.");
	//COMPARE(adapter->getExtTorque(), n_ext, "External torque was changed.");


	// check joint forces
	for (unsigned int i = 0; i < numJoints; i++){
		iToStr(i, jointNumberBuffer);
		COMPARE(adapter->getJointForce(i), joints_tau.at(i), "Joint " + jointNumberBuffer + " force wrong.");
	};
	// also check TCP props (which are usually calculated by the seperatly tested dirkin algo)
	COMPARE(adapter->getTCPPos(), tcp_r, "TCP position wrong.");
  COMPARE(adapter->getTCPOri(), tcp_R, "TCP orientation wrong.");
	COMPARE(adapter->getTCPVel(), tcp_v, "TCP velocity wrong.");
	COMPARE(adapter->getTCPAngVel(), tcp_omega, "TCP angular velocity wrong.");
	COMPARE(adapter->getTCPAcc(), tcp_vdot, "TCP acceleration wrong.");
	COMPARE(adapter->getTCPAngAcc(), tcp_omegadot, "TCP angular acceleration wrong.");


	// give general feedback for the invdyn algo
	if (testFailed) LOGOUT("      <invdyn method=\"" << algorithmName << "\" ok=\"0\">" << errorsDescription << "</invdyn>")
	else LOGOUT("      <invdyn method=\"" << algorithmName << "\" ok=\"1\"/>");
	GEN_DEBUG_MODEL_VALUES
};

void Tester::checkDirDyn(std::string algorithmName)
{
	if (algorithmName == "") return; // if the algorithm was not run skip the checks (also no log outputs in this case)
	std::string errorsDescription = "";
	std::string compareResult = "";
	std::string jointNumberBuffer = "";

	// check whether set props were left untouched
	// TODO: change text in case of a free base (then the problem is not the change but the value)
	COMPARE(adapter->getBasePos(), base_r, "Base position was changed.")
	COMPARE(adapter->getBaseOri(), base_R, "Base orientation was changed.")
	COMPARE(adapter->getBaseVel(), base_v, "Base velocity was changed.");
	COMPARE(adapter->getBaseAngVel(), base_omega, "Base angular velocity was changed.");
	COMPARE(adapter->getBaseAcc(), base_vdot, "Base acceleration was changed.");
	COMPARE(adapter->getBaseAngAcc(), base_omegadot, "Base angular acceleration was changed.");

	//COMPARE(adapter->getGravitiy(), gravitiy, "Gravity was changed.");
	
	for (unsigned int i = 0; i < numJoints; i++){
		iToStr(i, jointNumberBuffer);
		COMPARE(adapter->getJointPos(i), joints_q.at(i), "Joint " + jointNumberBuffer + " position was changed.");
		COMPARE(adapter->getJointVel(i), joints_qdot.at(i), "Joint " + jointNumberBuffer + " velocity was changed.");
		COMPARE(adapter->getJointForce(i), joints_tau.at(i), "Joint " + jointNumberBuffer + + " force was changed.");
	};

	//COMPARE(adapter->getExtForce(), f_ext, "External force was changed.");
	//COMPARE(adapter->getExtTorque(), n_ext, "External torque was changed.");


	// check joint accelerations
	for (unsigned int i = 0; i < numJoints; i++){
		iToStr(i, jointNumberBuffer);
		COMPARE(adapter->getJointAcc(i), joints_qdotdot.at(i), "Joint " + jointNumberBuffer + " acceleration wrong.");
	};
	// also check TCP props (which are usually calculated by the seperatly tested dirkin algo)
	COMPARE(adapter->getTCPPos(), tcp_r, "TCP position wrong.");
	COMPARE(adapter->getTCPOri(), tcp_R, "TCP orientation wrong.");
	COMPARE(adapter->getTCPVel(), tcp_v, "TCP velocity wrong.");
	COMPARE(adapter->getTCPAngVel(), tcp_omega, "TCP angular velocity wrong.");
	COMPARE(adapter->getTCPAcc(), tcp_vdot, "TCP acceleration wrong.");
	COMPARE(adapter->getTCPAngAcc(), tcp_omegadot, "TCP angular acceleration wrong.");


	// give general feedback for the dirdyn algo
	if (testFailed) LOGOUT("      <dirdyn method=\"" << algorithmName << "\" ok=\"0\">" << errorsDescription << "</dirdyn>")
	else LOGOUT("      <dirdyn method=\"" << algorithmName << "\" ok=\"1\"/>");
	GEN_DEBUG_MODEL_VALUES
};

std::string Tester::checkHybDyn(const std::vector<bool> & directJointList){
	//TODO: process directJointList in errorDescription (i.e. to be able to know which hybrid case failed)
	std::string errorsDescription = "";
	std::string compareResult = "";
	std::string jointNumberBuffer = "";

	// check whether set props were left untouched
	// TODO: change text in case of a free base (then the problem is not the change but the value)
	COMPARE(adapter->getBasePos(), base_r, "Base position was changed.")
	COMPARE(adapter->getBaseOri(), base_R, "Base orientation was changed.")
	COMPARE(adapter->getBaseVel(), base_v, "Base velocity was changed.");
	COMPARE(adapter->getBaseAngVel(), base_omega, "Base angular velocity was changed.");
	COMPARE(adapter->getBaseAcc(), base_vdot, "Base acceleration was changed.");
	COMPARE(adapter->getBaseAngAcc(), base_omegadot, "Base angular acceleration was changed.");

	//COMPARE(adapter->getGravitiy(), gravitiy, "Gravity was changed.");

	for (unsigned int i = 0; i < numJoints; i++){
		iToStr(i, jointNumberBuffer);
		COMPARE(adapter->getJointPos(i), joints_q.at(i), "Joint " + jointNumberBuffer + " position was changed.");
		COMPARE(adapter->getJointVel(i), joints_qdot.at(i), "Joint " + jointNumberBuffer + " velocity was changed.");
	};

	//COMPARE(adapter->getExtForce(), f_ext, "External force was changed.");
	//COMPARE(adapter->getExtTorque(), n_ext, "External torque was changed.");


	for (unsigned int i = 0; i < numJoints; i++){
		iToStr(i, jointNumberBuffer);
		if (directJointList.at(i) == true) {
		  // check joint accelerations for dir dyn joints
		  COMPARE(adapter->getJointForce(i), joints_tau.at(i), "Joint " + jointNumberBuffer + + " force was changed.");
		  COMPARE(adapter->getJointAcc(i), joints_qdotdot.at(i), "Joint " + jointNumberBuffer + " acceleration wrong.");
		} else {
		  // check joint forces for inv dyn joints
		  COMPARE(adapter->getJointAcc(i), joints_qdotdot.at(i), "Joint " + jointNumberBuffer + " acceleration was changed.");
		  COMPARE(adapter->getJointForce(i), joints_tau.at(i), "Joint " + jointNumberBuffer + + " force wrong.");
	  };
	};

	// also check TCP props
	COMPARE(adapter->getTCPPos(), tcp_r, "TCP position wrong.");
	COMPARE(adapter->getTCPOri(), tcp_R, "TCP orientation wrong.");
	COMPARE(adapter->getTCPVel(), tcp_v, "TCP velocity wrong.");
	COMPARE(adapter->getTCPAngVel(), tcp_omega, "TCP angular velocity wrong.");
	COMPARE(adapter->getTCPAcc(), tcp_vdot, "TCP acceleration wrong.");
	COMPARE(adapter->getTCPAngAcc(), tcp_omegadot, "TCP angular acceleration wrong.");

	// return feedback for the hybdyn algo
	return errorsDescription; // ="" if test succeded
	//GEN_DEBUG_MODEL_VALUES
};