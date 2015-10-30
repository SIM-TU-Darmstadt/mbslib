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

#include "GenericRobotModelReader.h"
//#include "RPYSensor.h"
#include "parseTools.h"

#include <fstream>
#include <string>
#include <sstream>
#include <string.h>

//#include <murosimf/core/tools/ErrorLogger.h>
//-------
#include <iostream>
//TODO: seperate errorLogger for reader
#define LOGOUT(X) std::cout << "..logout: " << X << std::endl;
#define WARNOUT(X) LOGOUT(X)
#define ERROUT(X) LOGOUT(X)
#define ALLOUT(X) LOGOUT(X)
//-------

using namespace robotmodelreaderLib;

GenericRobotModelReader::GenericRobotModelReader()
{
	// init mass and inertia buffers
	this->m = 0;
	this->i11 = 0;
	this-> i11 = 0;
	this-> i12 = 0;
	this-> i13 = 0;
	this-> i21 = 0;
	this-> i22 = 0;
	this-> i23 = 0;
	this-> i31 = 0;
	this-> i32 = 0;
	this-> i33 = 0;
}


void GenericRobotModelReader::readRobotModel(const std::string & s)
{
  std::stringstream ss;
  ss.str(s);
  errorTrace.push_back("; constant input string");
  readRobotModel(ss);// does imply cleanup
  errorTrace.pop_back();
}


void GenericRobotModelReader::readRobotModelFromFile(const std::string & filename, const char * name)
{  
	std::stringstream ss;

	std::string completeFilename = filePrefix + filename;

	std::ifstream file(filename.c_str());
	if(!file.is_open()){
		ss << "Error opening file " << completeFilename << std::endl;
		setError(FILE_ERROR, ss.str());
		cleanupFailedReaderRun();
		return;
	}

	ss << "; file " << completeFilename;
	errorTrace.push_back(ss.str());
	readRobotModel(file,name);// does imply cleanup
	errorTrace.pop_back();
}



void GenericRobotModelReader::readRobotModel(std::istream & is,const char * name)
{
  std::string name_;
  if(name){name_ = name;}

	int rv_prep = prepareReaderRun(name_);
	// error handling for preparation errors:
	if (rv_prep) {
		cleanupFailedReaderRun();
		return;
	}

  //parse the description
  int rv = protReadRobotModel(is);

  //check if there were parse errors
  if(rv){
		cleanupFailedReaderRun();
		return;
  }

  //check if model is valid
  if(!isValidModel()){
		setError(ROBOT_NOT_VALID, "Robot not valid");
		cleanupFailedReaderRun();
		return;
  }

  //all is well
  setError(OK);
	cleanupSuccessfulReaderRun();
}

int GenericRobotModelReader::setError(int lastError, std::string lastErrorString)
{
	this->lastError = lastError;
	this->lastErrorString = lastErrorString;
	return lastError;
}

int GenericRobotModelReader::protReadRobotModel(std::istream & is)
{
  int lineNr = 0;
  while(is.good()){
    if(is.eof()){
      return 0;
    }
    
    std::string line;
    std::getline(is,line);
		trim(line);
    lineNr++;
    LOGOUT( "RobotModelReader::protReadRobotModel -- parsing line Nr " <<  lineNr << " ...");
    addLineNumberToErrorTrace(lineNr);
    int rv = parseLine(line);
    if( rv ){
      generateErrorString(rv);
      ERROUT( "RobotModelReader::protReadRobotModel -- error " << rv << " parsing line nr " << lineNr << " : " << lastErrorString );
      return rv;
    }
    errorTrace.pop_back();
    LOGOUT( "  ... line ok" );
    if(is.eof()){
      return 0;
    }
  }
  //there was some error while reading
  ERROR(FILE_ERROR);
}


int GenericRobotModelReader::parseLine(std::string & line)
{
  //if line is empty, ignore the line
  if(line.length() == 0){
    return 0;
  }

  //if first character of line is %, ignore the line
  if(line.at(0) == '%'){
    return 0;
  }

  LOGOUT( "  RobotModelReader::parseLine '" << line << "'" );
  // expand macros
  if(expand(line)){
    LOGOUT( "  RobotModelReader::parseLine expanded to:'" << line << "'" );
  }
  
  //tokenize line
  std::vector<std::string> tokens;
  int rv = tokenize(line,tokens);
  if(rv <= 0){
    return rv;
  }
  LOGOUT( "  RobotModelReader::parseLine tokenizing ok");
  
  return handleLine(tokens);
}


int GenericRobotModelReader::handleLine(std::vector<std::string> & tokens){
  CHECK_ADD_COMMAND(set);
  CHECK_ADD_COMMAND(include);

  CHECK_ADD_COMMAND(fixedbase);
  CHECK_ADD_COMMAND(freebase);
  CHECK_ADD_COMMAND(fork);
  CHECK_ADD_COMMAND(trans);
  CHECK_ADD_COMMAND(rot);
  CHECK_ADD_COMMAND(prisjoint);
  CHECK_ADD_COMMAND(rotjoint);
  CHECK_ADD_COMMAND(point);
  CHECK_ADD_COMMAND(rigidbody);

  CHECK_SET_COMMAND(mass);
  CHECK_SET_COMMAND(inertia);

  CHECK_SET_COMMAND(color);
  CHECK_SET_COMMAND(material);
  CHECK_SET_COMMAND(colmat);
  CHECK_SET_COMMAND(shape);
  CHECK_SET_COMMAND(mesh);
  
  CHECK_ADD_COMMAND(camera);
  CHECK_ADD_COMMAND(laserscanner);
  CHECK_ADD_COMMAND(servo);
  CHECK_ADD_COMMAND(sensor);

  CHECK_ADD_COMMAND(spring1D);
	CHECK_ADD_COMMAND(spring3D);

  //unknown command
  ERROR(UNKNOWN_COMMAND);
}


int GenericRobotModelReader::expand(std::string & line)
{
  unsigned int pos = 0;
  int expandCount = 0;
  bool foundMacro = false;
  std::string currentMacro;
  std::string outline;
  
  while(pos < line.length()){
    char c = line.at(pos);
    if(c == '$'){
      foundMacro = true;
    } else {
      if(!foundMacro){
        outline.push_back(c);
      } else {
        if( (c == ' ') || (c == '\\') ){
          if(macros.find(currentMacro) == macros.end() ){
            ERROR(UNKNOWN_VARIABLE);
          } else {
            outline.append(macros[currentMacro]);
            if(c == ' '){outline.append(" ");}
            currentMacro.clear();
            foundMacro = false;
            expandCount++;
          }
        } else {
          currentMacro.push_back(c);
        }
      }
    }
    pos++;
  }
  if(foundMacro){
    if(macros.find(currentMacro) == macros.end() ){
      ERROR(UNKNOWN_VARIABLE);
    } else {
      outline.append(macros[currentMacro]);
      //outline.append(" ");
      currentMacro.clear();
      foundMacro = false;
      expandCount++;
    }
  }
  line = outline;
  return expandCount;
}


int GenericRobotModelReader::tokenize(const std::string &data, std::vector<std::string> &tokens)
{
	tokens.clear();
	unsigned int pos = 0;
	int tokCount = 0;
	bool foundtoken = false;
	std::string currentToken;
	while(pos < data.length()){
		if(data.at(pos) == ' '){
			if(foundtoken){
				tokens.push_back(currentToken);
				tokCount++;
				currentToken.clear();
				foundtoken = false;
			}
		} else {
			currentToken.push_back(data.at(pos));
			foundtoken = true;
		}
		pos++;
	}
	if(foundtoken){
		tokens.push_back(currentToken);
		tokCount++;
	}
	return tokCount;
}


int GenericRobotModelReader::set( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR_MIN(2);
  std::stringstream ss;
  std::vector<std::string>::iterator it = tokens.begin();
  it++;
  it++;
  while(it != tokens.end() ){
    ss << *it;
    it++;
    if(it != tokens.end()){ ss << " "; }
  }
  macros[tokens[1]] = ss.str();

  return 0;
}


int GenericRobotModelReader::include( std::vector<std::string> & tokens )
{
  UNNAMED;
  CHECK_TOKEN_NR(2);
  
  std::string completeFilename = filePrefix + tokens[1];
  
  std::ifstream ifile(completeFilename.c_str());
  
  if(!ifile.is_open()){
    ERROR(FILE_ERROR);
  }
  
  std::stringstream ss;
  ss << "; file " << tokens[1];
  
  errorTrace.push_back(ss.str());
  int rv = protReadRobotModel(ifile);
  errorTrace.pop_back();
  return rv;
}


int GenericRobotModelReader::fixedbase( std::vector<std::string> & tokens )
{
	ISNAMED;
	CHECK_TOKEN_NR(1);
	if( addFixedBase(LINKNAME) ){
		ERROR(COULDNT_ADD_ELEMENT);
	}
	return OK;
}


int GenericRobotModelReader::freebase( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR(1);
  if( addFreeBase(LINKNAME) ){
    ERROR(COULDNT_ADD_ELEMENT);
  }
  return OK;
}


int GenericRobotModelReader::fork( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR(1);
  if( addFork(LINKNAME) ){
    ERROR(COULDNT_ADD_ELEMENT);
  }
  return OK;
}


int GenericRobotModelReader::trans( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR(4);
  PARSE_FLOAT(x,1);
  PARSE_FLOAT(y,2);
  PARSE_FLOAT(z,3);

  if( addTranslation(x,y,z,LINKNAME) ){
    ERROR(COULDNT_ADD_ELEMENT);
  }
  return OK;
}


int GenericRobotModelReader::rot( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR(5);
  PARSE_FLOAT(x,1);
  PARSE_FLOAT(y,2);
  PARSE_FLOAT(z,3);
  PARSE_FLOAT(a,4);

  if( addRotation(x,y,z,a,LINKNAME) ){
    ERROR(COULDNT_ADD_ELEMENT);
  }
  return OK;
}


int GenericRobotModelReader::prisjoint( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR(4);
  PARSE_FLOAT(x,1);
  PARSE_FLOAT(y,2);
  PARSE_FLOAT(z,3);

  if( addPrismaticJoint(x,y,z,LINKNAME) ){
    ERROR(COULDNT_ADD_ELEMENT);
  }
  return OK;
}


int GenericRobotModelReader::rotjoint( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR(4);
  PARSE_FLOAT(x,1);
  PARSE_FLOAT(y,2);
  PARSE_FLOAT(z,3);

  if( addRotationalJoint(x,y,z,LINKNAME) ){
    ERROR(COULDNT_ADD_ELEMENT);
  }
  return OK;
}


int GenericRobotModelReader::point( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR(1);

  if( addPoint(this->m, this->i11, this->i12, this->i13, this->i21, this->i22, this->i23, this->i31, this->i32, this->i33, LINKNAME) ){
    ERROR(COULDNT_ADD_ELEMENT);
  }
	// reset buffer initials to 0
	this->m = 0;
	this->i11 = 0;
	this-> i11 = 0;
	this-> i12 = 0;
	this-> i13 = 0;
	this-> i21 = 0;
	this-> i22 = 0;
	this-> i23 = 0;
	this-> i31 = 0;
	this-> i32 = 0;
	this-> i33 = 0;

  return OK;
}

int GenericRobotModelReader::rigidbody( std::vector<std::string> & tokens )
{
	// parameters (all optional): r, com, mass, inertia
	ISNAMED;
	CHECK_TOKEN_NR_MIN(1);
	CHECK_TOKEN_NR_MAX(18);

	PARSE_FLOAT_OPT(r1,1,0);
	PARSE_FLOAT_OPT(r2,2,0);
	PARSE_FLOAT_OPT(r3,3,0);
	//TesterVector3 r(r1, r2, r3);

	PARSE_FLOAT_OPT(com1,4,0);
	PARSE_FLOAT_OPT(com2,5,0);
	PARSE_FLOAT_OPT(com3,6,0);
	//TesterVector3 com(com1, com2, com3);

	PARSE_FLOAT_OPT(m,7,0);

	PARSE_FLOAT_OPT(i11,8,0);
	PARSE_FLOAT_OPT(i12,9,0);
	PARSE_FLOAT_OPT(i13,10,0);
	PARSE_FLOAT_OPT(i21,11,0);
	PARSE_FLOAT_OPT(i22,12,0);
	PARSE_FLOAT_OPT(i23,13,0);
	PARSE_FLOAT_OPT(i31,14,0);
	PARSE_FLOAT_OPT(i32,15,0);
	PARSE_FLOAT_OPT(i33,16,0);


	const unsigned int arg_offset = 8;
	if( TOKEN_NR_COUNT == 1 + arg_offset ){
		// value for all diagonal elements (everthing else = 0)
		i22 = i11; // m11 already present
		i33 = i11;
	} else if ( TOKEN_NR_COUNT == 3 + arg_offset ) {
		// values for diagonal present in m11,m12,m13
		i22 = i12; i12 = 0;
		i33 = i13; i13 = 0;
	} else if ( TOKEN_NR_COUNT == 6 + arg_offset ) {
		// values for diagonal present in  m11,m12,m13 and for the rest in m21,m22,m23
		// backup m22
		float i22_old = i22;
		i22 = i12;
		i33 = i13;

		i12 = i21;
		i13 = i22_old;
		// m23 already correct

		// for symmetrie:
		// m21 already present and correct
		i31 = i13;
		i32 = i23;
	} else if ( TOKEN_NR_COUNT == 9 + arg_offset ) {
		// nothing to do all values already present from the correct position
	} else if ( TOKEN_NR_COUNT <= arg_offset) {
		// no inertia specified
	} else {
		ERROUT("Wrong number of arguments for inertia - allowed: 1, 3, 6 or 9 - but is: " << TOKEN_NR_COUNT - arg_offset);
		ERROR(WRONG_NUMBER_OF_ARGUMENTS); 
	}

	// check for symmetry
	if (CHECK_NOT_SYMMETRIC(i)) {
		ERROUT("Inertia has to be symmetric")
			ERROR(ILLEGAL_VALUE)
	}

	// check for positive definite (via principal minors) - but accept zero matrix
	if (CHECK_NOT_POSITIVE_DEFINIT(i)) {
		ERROUT("Inertia has to be positive definit")
			ERROR(ILLEGAL_VALUE)
	}



// 	TesterScalar i[9];
// 	i[0] = i11;
// 	i[1] = i12;
// 	i[2] = i13;
// 	i[3] = i21;
// 	i[4] = i22;
// 	i[5] = i23;
// 	i[6] = i31;
// 	i[7] = i32;
// 	i[8] = i33;


	addRigidbody(r1, r2, r3, com1, com2, com3, m, i11, i12, i13, i21, i22, i23, i31, i32, i33, LINKNAME);

	return 0;
}



int GenericRobotModelReader::mass( std::vector<std::string> & tokens )
{
  UNNAMED;
  CHECK_TOKEN_NR(2);
  PARSE_FLOAT(m,1);

	if (this->m != 0) return VALUE_ALLREADY_PRESENT;
	// buffer mass for use with point
	this->m = m;
	return OK;
}

int GenericRobotModelReader::inertia( std::vector<std::string> & tokens )
{
  UNNAMED;
	CHECK_TOKEN_NR_MIN(2);
	CHECK_TOKEN_NR_MAX(10);
  // preload inertia with zeros
  PARSE_FLOAT(m11,1);
  PARSE_FLOAT_OPT(m12,2,0);
  PARSE_FLOAT_OPT(m13,3,0);
  PARSE_FLOAT_OPT(m21,4,0);
  PARSE_FLOAT_OPT(m22,5,0);
  PARSE_FLOAT_OPT(m23,6,0);
  PARSE_FLOAT_OPT(m31,7,0);
  PARSE_FLOAT_OPT(m32,8,0);
  PARSE_FLOAT_OPT(m33,9,0);

	const unsigned int arg_offset = 1;
	if( TOKEN_NR_COUNT == 1 + arg_offset ){
		// value for all diagonal elements (everthing else = 0)
		m22 = m11; // m11 already present
		m33 = m11;
	} else if ( TOKEN_NR_COUNT == 3 + arg_offset ) {
		// values for diagonal present in m11,m12,m13
		m22 = m12; m12 = 0;
		m33 = m13; m13 = 0;
	} else if ( TOKEN_NR_COUNT == 6 + arg_offset ) {
		// values for diagonal present in  m11,m12,m13 and for the rest in m21,m22,m23
		// backup m22
		float m22_old = m22;
		m22 = m12;
		m33 = m13;

		m12 = m21;
		m13 = m22_old;
		// m23 already correct

		// for symmetrie:
		// m21 already present and correct
		m31 = m13;
		m32 = m23;
	} else if ( TOKEN_NR_COUNT == 9 + arg_offset ) {
		// nothing to do all values already present from the correct position
	} else {
		ERROUT("Wrong number of arguments - allowed: 1, 3, 6 or 9 - but is: " << TOKEN_NR_COUNT - arg_offset);
		ERROR(WRONG_NUMBER_OF_ARGUMENTS);
	}

	// check for symmetry
	if (CHECK_NOT_SYMMETRIC(m)) {
		ERROUT("Inertia has to be symmetric")
		ERROR(ILLEGAL_VALUE)
	}

	// check for positive definite (via principal minors) - but accept zero matrix
	if (CHECK_NOT_POSITIVE_DEFINIT(m)) {
		ERROUT("Inertia has to be positive definit")
		ERROR(ILLEGAL_VALUE)
	}
	
	if (!(this->i11 == 0 && this->i12 == 0 && this->i13 == 0 && this->i21 == 0 && this->i22 == 0 && this->i23 == 0 && this->i31 == 0 && this->i32 == 0 && this->i33 == 0)) return VALUE_ALLREADY_PRESENT;

	// buffer inertia for use with point
	this->i11 = m11;
	this->i12 = m12;
	this->i13 = m13;
	this->i21 = m21;
	this->i22 = m22;
	this->i23 = m23;
	this->i31 = m31;
	this->i32 = m32;
	this->i33 = m33;

	return OK;
}


int GenericRobotModelReader::color( std::vector<std::string> & tokens )
{
  UNNAMED;
  CHECK_TOKEN_NR_MIN(4);
  CHECK_TOKEN_NR_MAX(9);
  PARSE_FLOAT(red,1);
  PARSE_FLOAT(green,2);
  PARSE_FLOAT(blue,3);
  PARSE_FLOAT_OPT(diffuse,4,0.7f);
  PARSE_FLOAT_OPT(ambient,5,0.3f);
  PARSE_FLOAT_OPT(spec,6,0);
  PARSE_FLOAT_OPT(shine,7,0);
  PARSE_FLOAT_OPT(alpha,8,1.0);

	int rv = setColor(red, green, blue, diffuse, ambient, spec, shine, alpha);
  if (rv) { ERROR(rv) }	else {return OK;};
}


int GenericRobotModelReader::material( std::vector<std::string> & tokens )
{
  return color(tokens);
}


int GenericRobotModelReader::colmat( std::vector<std::string> & tokens )
{
  UNNAMED;
  CHECK_TOKEN_NR(2);
  
	int rv = setCollisionMaterial(tokens.at(1));
	if (rv) { ERROR(rv) }	else {return OK;};
}


int GenericRobotModelReader::shape( std::vector<std::string> & tokens )
{
	UNNAMED;
	CHECK_TOKEN_NR_MIN(3);
	CHECK_TOKEN_NR_MAX(5);

	const int postitionOfParameterX = 2;
	PARSE_FLOAT(x,postitionOfParameterX);
	PARSE_FLOAT_OPT(y,postitionOfParameterX+1,0);
	PARSE_FLOAT_OPT(z,postitionOfParameterX+2,0);

	unsigned int tokenCountWithoutName = tokens.size() - (named?1:0);
	int rv = setShape(tokens[1], tokenCountWithoutName - postitionOfParameterX, x, y, z);
	if (rv) { ERROR(rv) }	else {return rv;};
}


int GenericRobotModelReader::mesh( std::vector<std::string> & tokens )
{
  UNNAMED;
  CHECK_TOKEN_NR_MIN(2);
  CHECK_TOKEN_NR_MAX(3);
  
  std::string completeFilename = filePrefix + tokens[1];
  
	int rv = OK;
	if(tokens.size() == 2){
		rv = setMesh(completeFilename);
	} else {
		rv = setMesh(completeFilename, tokens[2]);
	}
  if (rv) { ERROR(rv) }	else {return rv;};
}


int GenericRobotModelReader::camera( std::vector<std::string> & tokens)
{
  ISNAMED;
  CHECK_TOKEN_NR_MIN(5);
  CHECK_TOKEN_NR_MAX(7);
  
  PARSE_STRING(mountPointName,1);
  PARSE_INT(width,2);
  PARSE_INT(height,3);
  PARSE_FLOAT(horizontalAngle,4);
  PARSE_FLOAT_OPT(near,5,0.1f);
  PARSE_FLOAT_OPT(far,6,20.f);

	int rv = attachCamera(mountPointName,width,height,horizontalAngle,near,far,LINKNAME);
	if (rv) { ERROR(rv) }	else {return OK;};
}


int GenericRobotModelReader::laserscanner( std::vector<std::string> & tokens)
{
	ISNAMED;
	CHECK_TOKEN_NR_MIN(6);
	CHECK_TOKEN_NR_MAX(8);

	PARSE_STRING(mountPointName,1);
	PARSE_FLOAT(minRayLen,2);
	PARSE_FLOAT(maxRayLen,3);
	PARSE_FLOAT(horizontalAngle,4);
	PARSE_INT(horizontalRays,5);
	PARSE_FLOAT_OPT(verticalAngle,6,0);
	PARSE_INT_OPT(verticalRays,7,1);

	int rv = attachLaserscanner(mountPointName,minRayLen, maxRayLen, horizontalAngle, horizontalRays, verticalAngle, verticalRays, LINKNAME);
	if (rv) { ERROR(rv) }	else {return OK;};
}


int GenericRobotModelReader::servo( std::vector<std::string> & tokens)
{
  ISNAMED;
  CHECK_TOKEN_NR(4);
  
  PARSE_STRING(mountPointName,1);
  PARSE_INT(servoId,2);
  PARSE_INT(positionsalServo,3);
  
	int rv = attachServo(mountPointName, servoId, positionsalServo, LINKNAME);
	if (rv) { ERROR(rv) }	else {return OK;};
}


int GenericRobotModelReader::sensor( std::vector<std::string> & tokens)
{
  ISNAMED;
  CHECK_TOKEN_NR_MIN(4);
  
  PARSE_STRING(mountPointName,1);
  PARSE_INT(sensorID,2);
  PARSE_STRING(sensorType,3);
  
	int rv;
	if( ( sensorType == "gyro" ) || ( sensorType == "acc" ) || ( sensorType == "vel" ) || ( sensorType == "pos" ) ){
		CHECK_TOKEN_NR_MIN(7);
		CHECK_TOKEN_NR_MAX(11);
		PARSE_FLOAT(x,4);
		PARSE_FLOAT(y,5);
		PARSE_FLOAT(z,6);
		PARSE_FLOAT_OPT(min,7,-30);
		PARSE_FLOAT_OPT(max,8,30);
		PARSE_INT_OPT(minDigital,9,0x0000);
		PARSE_INT_OPT(maxDigital,10,0x0fff);
		rv = attachSensor(mountPointName,sensorID,sensorType,min,max,minDigital,maxDigital, LINKNAME, x,y,z);
	} else if ( ( sensorType == "yaw" ) || ( sensorType == "roll" ) || ( sensorType == "pitch" ) ) {
		CHECK_TOKEN_NR_MAX(8);
		PARSE_FLOAT_OPT(min,4,-30);
		PARSE_FLOAT_OPT(max,5,30);
		PARSE_INT_OPT(minDigital,6,0x0000);
		PARSE_INT_OPT(maxDigital,7,0x0fff);
		rv = attachSensor(mountPointName,sensorID,sensorType,min,max,minDigital,maxDigital, LINKNAME);
	} else {
		ERROUT("Unknown sensor type " << sensorType);
		ERROR(ILLEGAL_VALUE); 
	}

	if(rv == ALLOCATION_ERROR){
		ERROUT("Couldn't allocate sensor");
		ERROR(ALLOCATION_ERROR);
	}

  if (rv) { ERROR(rv) }	else {return OK;};
}

int GenericRobotModelReader::spring1D(std::vector<std::string> & tokens)
{
	ISNAMED;
	CHECK_TOKEN_NR_MIN(2);

	PARSE_STRING(springModelType,1);

	int rv;
	if( ( springModelType == "linear" ) ){
		CHECK_TOKEN_NR_MIN(6);
		CHECK_TOKEN_NR_MAX(6);
		PARSE_STRING(driveJoint,2);
		PARSE_STRING(outputJoint,3);
		PARSE_FLOAT(k,4);
		PARSE_FLOAT(d,5);
		rv = addSpring1D(springModelType,driveJoint,outputJoint, LINKNAME, k, d);
	} else if ( ( springModelType == "linearWithRope" ) ) {
		CHECK_TOKEN_NR_MIN(7);
		CHECK_TOKEN_NR_MAX(7);
		PARSE_STRING(driveJoint,2);
		PARSE_STRING(outputJoint,3);
		PARSE_FLOAT(k,4);
		PARSE_FLOAT(d,5);
		PARSE_FLOAT(ropeLength,6);
		rv = addSpring1D(springModelType,driveJoint,outputJoint, LINKNAME, k, d,ropeLength);
	} else {
		ERROUT("Unknown 1D spring model type " << springModelType);
		ERROR(ILLEGAL_VALUE); 
	}

	if(rv == ALLOCATION_ERROR){
		ERROUT("Couldn't allocate spring");
		ERROR(ALLOCATION_ERROR);
	}

	if (rv) { ERROR(rv) }	else {return OK;};
}


int GenericRobotModelReader::spring3D(std::vector<std::string> & tokens)
{
	ISNAMED;
	CHECK_TOKEN_NR_MIN(3);

	PARSE_STRING(springModelType,1);
	PARSE_INT(numberOfEndpointsUsedBySpring,2);
	if (numberOfEndpointsUsedBySpring < 2){
		ERROUT("Each Spring3D must have at least two endpoints, not " << numberOfEndpointsUsedBySpring);
		ERROR(ILLEGAL_VALUE);
	}

	int rv;

	// common code for all springmodels
	CHECK_TOKEN_NR_MIN(3 + numberOfEndpointsUsedBySpring);
	std::string endPoint;
	std::vector<std::string> endPoints;
	for(int i = 3; i < 3 + numberOfEndpointsUsedBySpring; i++){ //it is ok for i to be signed, because it is checked above
		PARSE_PREEXISTING_STRING(endPoint,i);
		endPoints.push_back(endPoint);
	}
	// springmodel specific code
	if( ( springModelType == "linear" ) ){
		CHECK_TOKEN_NR_MIN(5 + numberOfEndpointsUsedBySpring);
		CHECK_TOKEN_NR_MAX(5 + numberOfEndpointsUsedBySpring);
		PARSE_FLOAT(k,3 + numberOfEndpointsUsedBySpring);
		PARSE_FLOAT(d,3 + numberOfEndpointsUsedBySpring + 1);
		rv = addSpring3D(springModelType,endPoints, LINKNAME, k, d);
	} else if ( ( springModelType == "linearWithRope" ) ) {
		CHECK_TOKEN_NR_MIN(6 + numberOfEndpointsUsedBySpring);
		CHECK_TOKEN_NR_MAX(6 + numberOfEndpointsUsedBySpring);
		PARSE_FLOAT(k,3 + numberOfEndpointsUsedBySpring);
		PARSE_FLOAT(d,3 + numberOfEndpointsUsedBySpring + 1);
		PARSE_FLOAT(ropeLength, 3 + numberOfEndpointsUsedBySpring + 2);
		rv = addSpring3D(springModelType,endPoints, LINKNAME, k, d,ropeLength);
	} else {
		ERROUT("Unknown Spring3D model type " << springModelType);
		ERROR(ILLEGAL_VALUE); 
	}

	if(rv == ALLOCATION_ERROR){
		ERROUT("Couldn't allocate spring");
		ERROR(ALLOCATION_ERROR);
	}

	if (rv) { ERROR(rv) }	else {return OK;};
}



void GenericRobotModelReader::addLineNumberToErrorTrace(int i)
{
  //char buf[255];
  //sprintf(buf,"in line %i ",i);
  //std::string s(buf);
  std::stringstream s;
  s << "in line " << i;
  errorTrace.push_back(s.str());
}


void GenericRobotModelReader::generateErrorString(int erno)
{
  std::stringstream ss;
  
  ss << "Error " << erno << " occurred ";

  for(std::vector<std::string>::reverse_iterator it = errorTrace.rbegin(); it != errorTrace.rend(); it++){
    ss << (*it);
  }
  
  lastErrorString = ss.str();

}