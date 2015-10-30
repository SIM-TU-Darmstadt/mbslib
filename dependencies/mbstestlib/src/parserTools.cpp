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

#include "parserTools.h"

#include <sstream>

#define SCALE_VALUE(UNIT, FACTOR) if(s.compare(UNIT) == 0){ f = f * (FACTOR); return 0;}

bool fileHasAdditionalContent( std::ifstream& file )
{
	std::string stringBuffer = "";

	std::streampos positionBuffer = file.tellg(); // remember current file-position (we'll need to seek back after checking for content)
	file >> stringBuffer;

	file.clear(); // reset any eof (and similar) bits
	file.seekg(positionBuffer); // seek back to the file-position from the 

	return !(stringBuffer.empty());
};

std::string& trim( std::string& string ){
  //TODO: really trim both beginning and end
  if ((string.length() > 0) && (string.at(string.length()-1) == '\r')){
    string.erase(string.length()-1);
  }
	// clean up whitespaces and tabs from line-start
	for (unsigned int i=0; i < string.length(); i++) {
		if (string.at(i) == ' ' || string.at(i) == '\t'){
			string.erase(i);
		}	else {
			break; // stop as soon as something different from whitespaces have been found
		}
	}
	// clean up whitespaces and tabs from line-endling
	for (int i=string.length()-1; i >= 0; i--) {
		if (string.at(i) == ' ' || string.at(i) == '\t'){	
			string.erase(i);
		}	else {
			break; // stop as soon as something different from whitespaces have been found
		}
	}
  return string;
};


int parseFloat(float & f,const std::string & st){
  std::stringstream ss(st);
  std::string s;
  
  //parse float
  ss >> f;
  //if string is not good, a parse error occured
  if(ss.bad()){ return -1;}
  
  //parse string after float
  ss >> s;
  //if string has no length, we are finished
  if(s.length() == 0){return 0;}

  SCALE_VALUE("g",1);
  SCALE_VALUE("kg",1000);
  SCALE_VALUE("m",1);
  SCALE_VALUE("mm",0.001f);
  SCALE_VALUE("rad",1);
  SCALE_VALUE("deg", 3.1415926535f / 180);
  
  return -1;
}


int parseFloatDefault(float & f, const std::string & s, float def){
	if (s == "_"){
		f = def;
		return 0;
	} else {
		return parseFloat(f,s);
	}
}

int parseInt(int & f,const std::string & st){
	std::stringstream ss(st);
	std::string s;

	//parse int
	ss >> f;
	//if string is not good, a parse error occured
	if(ss.bad()){ return -1;}

	//parse string after float
	ss >> s;
	//if string has no length, we are finished
	if(s.length() == 0){return 0;}

	return -1;
}

int parseIntDefault(int & f, const std::string & s, int def){
	if (s == "_"){
		f = def;
		return 0;
	} else {
		return parseInt(f,s);
	}
}
