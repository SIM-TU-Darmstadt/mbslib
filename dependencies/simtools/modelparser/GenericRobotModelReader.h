/*
 * Copyright (C) 2008, 2009
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
 * @file GenericRobotModelReader.h
 * Definition of robotmodelreaderLib::GenericRobotModelReader
 */

#ifndef __robotmodelreaderLib_GenericRobotModelReader_h__
#define __robotmodelreaderLib_GenericRobotModelReader_h__

#include <istream>
#include <map>
#include <vector>

namespace robotmodelreaderLib{

/** \class GenericRobotModelReader <GenericRobotModelReader.h>
 * A reader for robot-model descriptions (Framework-Generic).
 *
 */
class GenericRobotModelReader{
//friend class RobotModelReaderTest;
public:
  /**
   * Constructor.
	 */
  GenericRobotModelReader();

  /**
   * Destructor.
	 */
  virtual ~GenericRobotModelReader(){}

	/// Return codes of the methods of RobotModelReader.
	static enum errorCodes{
		OK = 0,
		UNKNOWN_COMMAND = -1,
		WRONG_NUMBER_OF_ARGUMENTS = -2,
		COULDNT_ADD_ELEMENT = -3,
		TOKEN_PARSE_ERROR = -4,
		ROBOT_NOT_VALID = -5,
		FILE_ERROR = -6,
		COULDT_SET_VALUE = -7,
		VALUE_ALLREADY_PRESENT = -8,
		ILLEGAL_VALUE = -9,
		UNKNOWN_VARIABLE = -10,
		ALLOCATION_ERROR = -11,
		UNKNOWN_MOUNTPOINT = -12,
		BUILDER_ERROR = -13,
		NOT_IMPLEMENTED = -14
	} errorcodes;

	/**
   * Get last error which occurred during reading the model.
   */
  int getLastError()const{return lastError;}
  
  /**
   * Get string-description of last error.
   */
  const std::string & getLastErrorString()const{return lastErrorString;}

protected:
  /**
   * Read RobotModel from a string.
   *
   * (Guarantees a prepareRobotModel call before any model-changing calls are made.
	 *  Also guarantees a cleanup-call.)
   */
  void readRobotModel(const std::string & s);
 
  /**
   * Read RobotModel from a file.
   *
   * (Guarantees a prepareRobotModel call before any model-changing calls are made.
	 *  Also guarantees a cleanup-call.)
   */
	void readRobotModelFromFile(const std::string & filename, const char * name=0);

  /**
   * Read RobotModel from an arbitrary stream.
   *
   * (Guarantees a prepareRobotModel call before any model-changing calls are made.
	 *  Also guarantees a cleanup-call.)
   */
  void readRobotModel(std::istream & is, const char * name = 0);
  
  /**
   * Set the prefix for loading files.
   */
  void setFilePrefix(const std::string & prefix);

	/**
   * Set last error which occurred during reading the model.
	 * And set string-description of last error (defaults to empty String).
   */
	int setError(int lastError, std::string lastErrorString="");

	/**
	 * Prepare model parsing.
   * 
   * @return: returns the error code (0 if none happened) - must use setError!
	 */
  virtual int prepareReaderRun(std::string name)=0;

	/**
	 * Clean up after we got an error.
	 */
	virtual void cleanupFailedReaderRun()=0;
	/**
	 * Clean up after we parsed the model successfully.
	 */
	virtual void cleanupSuccessfulReaderRun()=0;

	/**
	 * Check model validity.
	 */
	virtual bool isValidModel()=0;

	//TODO: comments
	virtual errorCodes addFixedBase(const std::string & linkname)=0;
	virtual errorCodes addFreeBase(const std::string & linkname)=0;
	virtual errorCodes addFork(const std::string & linkname)=0;
	virtual errorCodes addTranslation(const float x, const float y, const float z, const std::string & linkname)=0;	
	virtual errorCodes addRotation(const float x, const float y, const float z, const float a, const std::string & linkname)=0;
	virtual errorCodes addPrismaticJoint(const float x, const float y, const float z, const std::string & linkname)=0;
	virtual errorCodes addRotationalJoint(const float x, const float y, const float z, const std::string & linkname)=0;
	virtual errorCodes addPoint(const float m, const float m11, const float m12, const float m13, const float m21, const float m22, const float m23, const float m31, const float m32, const float m33, const std::string & linkname)=0;
	virtual errorCodes addRigidbody(const float r_x, const float r_y, const float r_z, const float com_x, const float com_y, const float com_z, const float m, const float i11, const float i12, const float i13, const float i21, const float i22, const float i23, const float i31, const float i32, const float i33, const std::string & linkname)=0;
	virtual errorCodes setColor(const float red,const float green,const float blue,const float diffuse,const float ambient,const float spec,const float shine,const float alpha)=0;
	virtual errorCodes setCollisionMaterial(const std::string & key)=0;
	virtual errorCodes setShape(const std::string & shapeType, const unsigned int parametersCount, const float x, const float y, const float z)=0;
	virtual errorCodes setMesh(const std::string & completeFilename, const std::string & name="")=0;
	virtual errorCodes attachCamera(const std::string & mountPointName, const int width, const int height, const float horizontalAngle, const float near, const float far, const std::string & linkname)=0;
	virtual errorCodes attachLaserscanner(const std::string & mountPointName, const float minRayLen, const float maxRayLen, const float horizontalAngle, const int horizontalRays, const float verticalAngle, const int verticalRays, const std::string & linkname)=0;
	virtual errorCodes attachServo(const std::string & mountPointName, const int servoId, const int positionsalServo, const std::string & linkname)=0;
	virtual errorCodes attachSensor(const std::string & mountPointName, const int sensorID, const std::string & sensorType, const float min, const float max, const int minDigital, const int maxDigital, const std::string & linkname, const float axis_x = 0, const float axis_y = 0, const float axis_z = 0)=0;
	virtual	errorCodes addSpring1D(const std::string & springModel, const std::string & driveJoint, const std::string & outputJoint,const std::string & name="", const float k=0, const float d=0, const float ropeLength=0)=0;
	virtual	errorCodes addSpring3D(const std::string & springModel, const std::vector<std::string> & endPoints, const std::string & name="", const float k=0, const float d=0, const float ropeLength=0)=0;

private:
  /**
   * Read robot model from an istream.
   */
  int protReadRobotModel(std::istream & is);
  
  /**
   * Parse one line of input.
   */
  int parseLine(std::string & line);
  
  /**
   * Handle tokenized line.
   */
  virtual int handleLine(std::vector<std::string> & tokens);
  
  /**
   * Expand a macros of a line.
   */
  int expand(std::string & line);
  
  /**
   * Tokenize a line.
   */
  int tokenize(const std::string & data, std::vector<std::string> & tokens);

  /**
   * Handle set command.
   */
  int set( std::vector<std::string> & tokens);
  
  /**
   * Handle include command.
   */
  int include( std::vector<std::string> & tokens);

  /**
   * Handle fixedbase command.
   */
  int fixedbase( std::vector<std::string> & tokens);
  
  /**
   * Handle freebase command.
   */
  int freebase( std::vector<std::string> & tokens);
  
  /**
   * Handle fork command.
   */
  int fork( std::vector<std::string> & tokens);
  
  /**
   * Handle trans command.
   */
  int trans( std::vector<std::string> & tokens);
  
  /**
   * Handle rot command.
   */
  int rot( std::vector<std::string> & tokens);
  
  /**
   * Handle prisjoint command.
   */
  int prisjoint( std::vector<std::string> & tokens);
  
  /**
   * Handle rotjoint command.
   */
  int rotjoint( std::vector<std::string> & tokens);
  
  /**
   * Handle point command.
   */
  int point( std::vector<std::string> & tokens);

	/**
   * Handle rigidbody command.
   */
	int rigidbody( std::vector<std::string> & tokens);

  /**
   * Handle mass command.
   */
  int mass( std::vector<std::string> & tokens);
  
  /**
   * Hanlde inertia command.
   */
  int inertia( std::vector<std::string> & tokens);

  /**
   * Hanlde color command.
   */
  int color( std::vector<std::string> & tokens);

  /**
   * Hanlde material command.
   */
  int material( std::vector<std::string> & tokens);
  
  /**
   * Handle collision material command.
   */
  int colmat( std::vector<std::string> & tokens);
  

  /**
   * Handle shape command.
   */
  int shape( std::vector<std::string> & tokens);
  
  /**
   * Handle mesh command.
   */
  int mesh( std::vector<std::string> & tokens);

  /**
   * Handle camera command.
   */
  int camera( std::vector<std::string> & tokens);
  
  /**
   * Handle servo command.
   */
  int servo( std::vector<std::string> & tokens);

	/**
   * Handle 1D spring command.
   */
  int spring1D( std::vector<std::string> & tokens);

	/**
   * Handle 3D spring command.
   */
  int spring3D( std::vector<std::string> & tokens);

	/**
   * Handle sensor command.
   */
  int sensor( std::vector<std::string> & tokens);

  /**
   * Handle laserscanner command.
   */
  int laserscanner( std::vector<std::string> & tokens);
  
  /**
   * Add a line number to the error trace.
   */
  void addLineNumberToErrorTrace(int i);
  
  /**
   * Generate an error string from the error trace.
   */
  void generateErrorString(int erno);

  /// Number of last error.
  int lastError;
  
  /// Last error string.
  std::string lastErrorString;
  
  /// Trace of parer to generate error messages from.
  std::vector<std::string> errorTrace;
  
  /// Map of macros registered.
  std::map<std::string,std::string> macros;
    
  /// The prefix for files.
  std::string filePrefix;

	/// Buffer to store mass
	float m;

	/// Buffer to store inertia
	float i11;
	float i12;
	float i13;
	float i21;
	float i22;
	float i23;
	float i31;
	float i32;
	float i33;

}; //class GenericRobotModelReader

} // namespace robotmodelreaderLib

#endif
