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
 * @file RobotModelParser.h
 * Declaration of the Parser for Robotermodels
 */

#ifndef ROBOTMODELPARSER_H_ 
#define ROBOTMODELPARSER_H_ 

#include "Adapter.hpp"
#include "TesterTypes.h"

#include <iostream>
#include <string>
#include <vector>
#include <map>

/** \class RobotModelParser <RobotModelParser.h>
 * Reads and parses a model to be later used by tests
 */
class RobotModelParser{
public:
  /**
   * Destructor.
   */
  virtual ~RobotModelParser(){}

  /**
   * Read RobotModel from an arbitrary stream.
   *
	 * @adapter: the tester-adapter to work on (i.e. use for model creation calls)
   * @return: returns the number of joints. -1 if an error occured.
   */
  int readRobotModel(std::istream & is, Adapter * adapter);

 /**
   * Get the model in String form (e.g. like in the testspec-file).
   */
  std::string getModelAsString()const{
		return modelAsStringCache;
	}


 /**
   * Get last error which occured during reading the model.
   */
  int getLastError()const{return lastError;}
  
  /**
   * Get string-description of last error.
   */
  const std::string & getLastErrorString()const{return lastErrorString;}

	/// Return codes of the methods of RobotModelReader.
  enum{
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
    ALLOCATION_ERROR = -11
  } errorcodes;

private:

  /**
   * Read robot model from an istream.
   */
  int protReadRobotModel(std::istream &); 

	/**
   * Parse one line of input.
   */
  int parseLine(std::string &);

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
   * Handle inertia command.
   */
  int inertia( std::vector<std::string> & tokens);

  /**
   * Handle color command.
   */
  int color( std::vector<std::string> & tokens);

  /**
   * Handle material command.
   */
  int material( std::vector<std::string> & tokens);
  
  /**
   * Handle shape command.
   */
  int shape( std::vector<std::string> & tokens);
  
  /**
   * Handle mesh command.
   */
  int mesh( std::vector<std::string> & tokens);

  /**
   * Handle endmodel command.
   */
  int endmodel( std::vector<std::string> & tokens);

  /**
   * Stores the info/state weather the model is parsed completely
   * so we can move on, exiting the model parser,
   * leaving the rest of the input for someone else to process
   */
  bool parsingComplete;



  /**
   * Add a line number to the error trace.
   */
  void addLineNumberToErrorTrace(int);
  
  /**
   * Generate an error string from the error trace.
   */
  void generateErrorString(int); 

	/// Number of last error.
  int lastError;
  
  /// Last error string.
  std::string lastErrorString;
  
  /// Trace of parer to generate error messages from.
  std::vector<std::string> errorTrace;
  
	/// The the model represented as a String (for logging purposes)
	std::string modelAsStringCache;

  /// Map of macros registered.
  std::map<std::string,std::string> macros;
  
  /// The adapter to create a robot with.
  Adapter * adapter;

	/// A counter for the number of joints in the model
	unsigned int numJoints;

	/// Buffer to store mass
	TesterScalar m;

	/// Buffer to store inertia
	TesterMatrix3x3 i;
  
}; // class RobotModelParser

#endif /* ROBOTMODELPARSER_H_ */
