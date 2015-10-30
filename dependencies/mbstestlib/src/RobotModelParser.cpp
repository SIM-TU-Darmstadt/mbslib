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
 * @file RobotModelParser.cpp
 * Definition of the Parser for Robotermodels
 */

#include "RobotModelParser.h"
#include "parserTools.h"
#include "ErrorLogger.h"
#include <sstream>
#include <fstream>

//using namespace mbslib;


int RobotModelParser::readRobotModel(std::istream & is, Adapter * adapter)
{
	// init numJoints counter
	numJoints = 0;
	// init mass and inertia buffers
	this->m = 0;
	this->i = TesterMatrix3x3();

	// init model string cache
	modelAsStringCache = "";

	// init parsing status
	parsingComplete = false;

	// remember adapter on which we create the model
	this->adapter = adapter;

  //parse the description
  int rv = protReadRobotModel(is);

  //check if there were parse errors
  if(rv){
    return -1;
  }

  //if all is well, return the model
  lastError = OK;
  return numJoints;
} 

int RobotModelParser::protReadRobotModel(std::istream & is)
{
  int lineNr = 0;
  while(is.good() && !parsingComplete){
    if(is.eof()){
      ERROR(FILE_ERROR);
    }
    
    std::string line;
    std::getline(is,line);
    trim(line);
    lineNr++;
		modelAsStringCache.append(line + "\n"); // cache the model in a String (for logging)
    //LOGOUT( "RobotModelReader::protReadRobotModel -- parsing line Nr " <<  lineNr << " ...");
    addLineNumberToErrorTrace(lineNr);
    int rv = parseLine(line);
    if( rv ){
      generateErrorString(rv);
      ERROUT( "RobotModelReader::protReadRobotModel -- error " << rv << " parsing line nr " << lineNr << " : " << lastErrorString );
      return rv;
    }
    errorTrace.pop_back();
    //LOGOUT( "  ... line ok");
    if (parsingComplete) return 0; // endmodel was reached so we can stop parsing
  }
  //there was some error while reading
  ERROR(FILE_ERROR);
	// remove last newline from modelstring cache
	if (modelAsStringCache.size() > 0) modelAsStringCache.resize(modelAsStringCache.size() - 1);
}

int RobotModelParser::parseLine(std::string & line)
{
  //if line is empty, ignore the line
  if(line.length() == 0){
    return 0;
  }

  //if first character of line is %, ignore the line
  if(line.at(0) == '%'){
    return 0;
  }

  //LOGOUT( "  RobotModelReader::parseLine '" << line << "'" );
  // expand macros
  if(expand(line)){
    //LOGOUT( "  RobotModelReader::parseLine expanded to:'" << line << "'" );
  }
  
  //std::cout << std::endl << " |" << line << "|" << std::endl;
  //tokenize line
  std::vector<std::string> tokens;
  int rv = tokenize(line,tokens);
  if(rv <= 0){
    return rv;
  }
  //LOGOUT( "  RobotModelReader::parseLine tokenizing ok");
  
  return handleLine(tokens);
} 

int RobotModelParser::handleLine(std::vector<std::string> & tokens){
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

  //CHECK_SET_COMMAND(color);
  //CHECK_SET_COMMAND(material);
  //CHECK_SET_COMMAND(shape);
  //CHECK_SET_COMMAND(mesh);

  CHECK_SET_COMMAND(endmodel);

  //unknown command
  ERROR(UNKNOWN_COMMAND);
}

int RobotModelParser::expand(std::string & line)
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

int RobotModelParser::tokenize(const std::string &data, std::vector<std::string> &tokens)
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






int RobotModelParser::set( std::vector<std::string> & tokens )
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

int RobotModelParser::include( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR(2);
  std::ifstream ifile(tokens[1].c_str());
  
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

int RobotModelParser::fixedbase( std::vector<std::string> & tokens )
{
  ISNAMED;
	CHECK_TOKEN_NR_MIN(1);
	CHECK_TOKEN_NR_MAX(13)
	// position r
	PARSE_FLOAT_OPT(r1,1,0)
	PARSE_FLOAT_OPT(r2,2,0)
	PARSE_FLOAT_OPT(r3,3,0)
	TesterVector3 r(r1, r2, r3);
	// orientation R
	PARSE_FLOAT_OPT(R11,4,1)
	PARSE_FLOAT_OPT(R12,5,0)
	PARSE_FLOAT_OPT(R13,6,0)
	PARSE_FLOAT_OPT(R21,7,0)
	PARSE_FLOAT_OPT(R22,8,1)
	PARSE_FLOAT_OPT(R23,9,0)
	PARSE_FLOAT_OPT(R31,10,0)
	PARSE_FLOAT_OPT(R32,11,0)
	PARSE_FLOAT_OPT(R33,12,1)
	TesterScalar R[9];
	R[0] = R11;
	R[1] = R12;
	R[2] = R13;
	R[3] = R21;
	R[4] = R22;
	R[5] = R23;
	R[6] = R31;
	R[7] = R32;
	R[8] = R33;

	adapter->addFixedBase(r,R);
  //if( builder->addFixedBase(LINKNAME) ){
  //  ERROR(COULDNT_ADD_ELEMENT);
  //}
  return 0;
}

int RobotModelParser::freebase( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR_MIN(1);
	CHECK_TOKEN_NR_MAX(13)
	// position r
	PARSE_FLOAT_OPT(r1,1,0)
	PARSE_FLOAT_OPT(r2,2,0)
	PARSE_FLOAT_OPT(r3,3,0)
	TesterVector3 r(r1, r2, r3);
	// orientation R
	PARSE_FLOAT_OPT(R11,4,1)
	PARSE_FLOAT_OPT(R12,5,0)
	PARSE_FLOAT_OPT(R13,6,0)
	PARSE_FLOAT_OPT(R21,7,0)
	PARSE_FLOAT_OPT(R22,8,1)
	PARSE_FLOAT_OPT(R23,9,0)
	PARSE_FLOAT_OPT(R31,10,0)
	PARSE_FLOAT_OPT(R32,11,0)
	PARSE_FLOAT_OPT(R33,12,1)
	TesterScalar R[9];
	R[0] = R11;
	R[1] = R12;
	R[2] = R13;
	R[3] = R21;
	R[4] = R22;
	R[5] = R23;
	R[6] = R31;
	R[7] = R32;
	R[8] = R33;

	adapter->addFreeBase(r,R);
  //if( builder->addFreeBase(LINKNAME) ){
  //  ERROR(COULDNT_ADD_ELEMENT);
  //}
  return 0;
}

int RobotModelParser::fork( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR(1);
	adapter->addFork();
  return 0;
}


int RobotModelParser::trans( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR(4);
  PARSE_FLOAT(x,1);
  PARSE_FLOAT(y,2);
  PARSE_FLOAT(z,3);

	adapter->addTranslation(TesterVector3(x,y,z),TesterVector3(-x,-y,-z),this->m, this->i); // the mass and inertia are always attached to the beginning of the translation

	// reset buffers
	this->m = 0;
	this->i = TesterMatrix3x3();

  //if( builder->addTranslation(x,y,z,LINKNAME) ){
  //  ERROR(COULDNT_ADD_ELEMENT);
  //}
  return 0;
}

int RobotModelParser::rot( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR(5);
  PARSE_FLOAT(x,1);
  PARSE_FLOAT(y,2);
  PARSE_FLOAT(z,3);
  PARSE_FLOAT(a,4);

	TesterVector3 axis = TesterVector3(x,y,z);
	axis.rescale2length1();

	adapter->addRotation(axis,a);
  //if( builder->addRotation(x,y,z,a,LINKNAME) ){
  //  ERROR(COULDNT_ADD_ELEMENT);
  //}
  return 0;
}

int RobotModelParser::prisjoint( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR(4);
  PARSE_FLOAT(x,1);
  PARSE_FLOAT(y,2);
  PARSE_FLOAT(z,3);

	this->numJoints++;
	adapter->addTranslationalJoint(TesterVector3(x,y,z));
  //if( builder->addPrismaticJoint(x,y,z,LINKNAME) ){
  //  ERROR(COULDNT_ADD_ELEMENT);
  //}
  return 0;
}

int RobotModelParser::rotjoint( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR(4);
  PARSE_FLOAT(x,1);
  PARSE_FLOAT(y,2);
  PARSE_FLOAT(z,3);

	TesterVector3 axis = TesterVector3(x,y,z);
	axis.rescale2length1();

	this->numJoints++;
	adapter->addRotationalJoint(axis);
  //if( builder->addRotationalJoint(x,y,z,LINKNAME) ){
  //  ERROR(COULDNT_ADD_ELEMENT);
  //}
  return 0;
}

int RobotModelParser::point( std::vector<std::string> & tokens )
{
  ISNAMED;
  CHECK_TOKEN_NR(1);

	adapter->addTranslation(TesterVector3(),TesterVector3(), this->m, this->i); // mass and inertia must have been stored in the buffer
	// TODO: add check, that we actually had a fork before (i.e. we have a branch to close)
	adapter->addEndPoint(LINKNAME); // close the branch (all following additions go the the other branch

	// reset buffer initials to 0
	this->m = 0;
	this->i = TesterMatrix3x3();
  //if( builder->addPoint(LINKNAME) ){
  //  ERROR(COULDNT_ADD_ELEMENT);
  //}
  return 0;
}

int RobotModelParser::rigidbody( std::vector<std::string> & tokens )
{
	// parameters (all optional): r, com, mass, inertia
	ISNAMED;
	CHECK_TOKEN_NR_MIN(1);
	CHECK_TOKEN_NR_MAX(18);

	PARSE_FLOAT_OPT(r1,1,0);
	PARSE_FLOAT_OPT(r2,2,0);
	PARSE_FLOAT_OPT(r3,3,0);
	TesterVector3 r(r1, r2, r3);

	PARSE_FLOAT_OPT(com1,4,0);
	PARSE_FLOAT_OPT(com2,5,0);
	PARSE_FLOAT_OPT(com3,6,0);
	TesterVector3 com(com1, com2, com3);

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



	TesterScalar i[9];
	i[0] = i11;
	i[1] = i12;
	i[2] = i13;
	i[3] = i21;
	i[4] = i22;
	i[5] = i23;
	i[6] = i31;
	i[7] = i32;
	i[8] = i33;

	
	adapter->addTranslation(r, com, m, TesterMatrix3x3(i));

	// if mass or inertia were set before (without being used this model is not valid)
	if( this->m != 0 || !(this->i.isEqual(TesterMatrix3x3()) )){
	  ERROR(ROBOT_NOT_VALID);
	}
	// reset buffer initials to 0
	this->m = 0;
	this->i = TesterMatrix3x3();
	return 0;
}

int RobotModelParser::mass( std::vector<std::string> & tokens )
{
  UNNAMED;
  CHECK_TOKEN_NR(2);
  PARSE_FLOAT(m,1);

	this->m = m; // buffer mass
  //switch(builder->setMass(m)){
  //  case -1 : ERROR(COULDT_SET_VALUE);
  //  case -2 : ERROR(VALUE_ALLREADY_PRESENT);
  //}
  return 0;
}

int RobotModelParser::inertia( std::vector<std::string> & tokens )
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

  
	TesterScalar ma[9];
	ma[0] = m11;
	ma[1] = m12;
	ma[2] = m13;
	ma[3] = m21;
	ma[4] = m22;
	ma[5] = m23;
	ma[6] = m31;
	ma[7] = m32;
	ma[8] = m33;

	this->i = TesterMatrix3x3(ma); // buffer inertia
	//TesterMatrix3x3 m;
	//m
  //TMatrix3x3 m;
  //m(1,1) = m11; m(1,2) = m12; m(1,3) = m13;
  //m(2,1) = m21; m(2,2) = m22; m(2,3) = m23;
  //m(3,1) = m31; m(3,2) = m32; m(3,3) = m33;

  //switch(builder->setInertia(m)){
  //  case -1 : ERROR(COULDT_SET_VALUE);
  //  case -2 : ERROR(VALUE_ALLREADY_PRESENT);
  //}
  return 0;

}

int RobotModelParser::color( std::vector<std::string> & tokens )
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

  //murosimf::vis::VisibleMaterial mat(red,green,blue,alpha,diffuse,ambient,spec,shine);
  //murosimf::vis::VhVisibleMaterial * vhVisMat;

  //builder->getCurrentObject()->getNamedValueHolder(vhVisMat);
  //if(vhVisMat){ERROR(VALUE_ALLREADY_PRESENT);}

  //builder->getCurrentObject()->requireNamedValueHolder(vhVisMat);
  //if(!vhVisMat){ERROR(COULDT_SET_VALUE);}
  //vhVisMat->getMutable() = mat;

  return 0;
}

int RobotModelParser::material( std::vector<std::string> & tokens )
{
  return color(tokens);
}

int RobotModelParser::shape( std::vector<std::string> & tokens )
{
  UNNAMED;
  CHECK_TOKEN_NR_MIN(3);
  CHECK_TOKEN_NR_MAX(5);

  PARSE_FLOAT(x,2);
  PARSE_FLOAT_OPT(y,3,0);
  PARSE_FLOAT_OPT(z,4,0);
  
  //PrimitiveShape s;

  if(tokens[1].compare("box") == 0){
    CHECK_TOKEN_NR(5);
    //s = PrimitiveShape(PrimitiveShape::PST_Box,x,y,z);
  } else if(tokens[1].compare("sphere") == 0){
    CHECK_TOKEN_NR(3);
    //s = PrimitiveShape(PrimitiveShape::PST_Sphere,2*x,2*x,2*x);
  } else if(tokens[1].compare("cylinder") == 0){
    CHECK_TOKEN_NR(4);
    //s = PrimitiveShape(PrimitiveShape::PST_Cylinder,2*x,2*x,y);
  } else if(tokens[1].compare("capsule") == 0){
    CHECK_TOKEN_NR(4);
    //s = PrimitiveShape(PrimitiveShape::PST_Capsule,2*x,2*x,y);
  } else {
    ERROR(ILLEGAL_VALUE);
  }

  //VhPrimitiveShape * vhPrimShape; 

  //builder->getCurrentObject()->getNamedValueHolder(vhPrimShape);
  //if(vhPrimShape){ERROR(VALUE_ALLREADY_PRESENT);}

  //builder->getCurrentObject()->requireNamedValueHolder(vhPrimShape);
  //if(!vhPrimShape){ERROR(COULDT_SET_VALUE);}
  //vhPrimShape->getMutable() = s;

  return 0;
}

int RobotModelParser::mesh( std::vector<std::string> & tokens )
{
  UNNAMED;
  CHECK_TOKEN_NR_MIN(2);
  CHECK_TOKEN_NR_MAX(3);
  
  std::string completeFilename = tokens[1];
  
  //MeshLoader * ml = meshLoaderCache.getMeshLoader(completeFilename);
  //if(!ml){ERROR(ILLEGAL_VALUE);}
  //
  //ITriangleMesh * m;
  //if(tokens.size() == 2){
  //  m = ml->getMesh("");
  //} else {
  //  m = ml->getMesh(tokens[2]);
  //  if(!m){ERROR(ILLEGAL_VALUE);}
  //}

  //VhMeshShape * vhMeshShape; 

  //builder->getCurrentObject()->getNamedValueHolder(vhMeshShape);
  //if(vhMeshShape){ERROR(VALUE_ALLREADY_PRESENT);}

  //builder->getCurrentObject()->requireNamedValueHolder(vhMeshShape);
  //if(!vhMeshShape){ERROR(COULDT_SET_VALUE);}
  //vhMeshShape->getMutable() = m;

  return 0;
}

int RobotModelParser::endmodel( std::vector<std::string> & tokens )
{
  UNNAMED;
  CHECK_TOKEN_NR(1);
  
  parsingComplete = true;

	// TODO: check for endpoint already present condition (eventually in the adapter finalizeModel method, if not here)
  if(adapter->finalizeModel()){
    return 0;
  } else {
    ERROR(ROBOT_NOT_VALID)
  };
}



void RobotModelParser::addLineNumberToErrorTrace(int i)
{
  //char buf[255];
  //sprintf(buf,"in line %i ",i);
  //std::string s(buf);
  std::stringstream s;
  s << "in line " << i;
  errorTrace.push_back(s.str());
}

void RobotModelParser::generateErrorString(int erno)
{
  std::stringstream ss;
  
  ss << "Error " << erno << " occurede ";

  for(std::vector<std::string>::reverse_iterator it = errorTrace.rbegin(); it != errorTrace.rend(); it++){
    ss << (*it);
  }
  
  lastErrorString = ss.str();

}