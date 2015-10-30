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
 * @file parseTools.h
 * Declaration of helper functions and macros for parsing of files.
 */
 

#ifndef __parserTools_h__
#define __parserTools_h__

#include <string>

#define READVECTOR(X) for (unsigned int i = 0; i < 3; i++){ testSpecFile >> X.vectorValues[i]; }
#define READMATRIX(X) for (unsigned int i = 0; i < 9; i++){ TesterScalar buffer[9]; testSpecFile >> buffer[i]; X = TesterMatrix3x3(buffer); }

// read Y values into the X-vector from testSpecFile
#define READARRAY(X,Y) {X.clear(); for (unsigned int i = 0; i < Y; i++){ TesterScalar buffer; testSpecFile >> buffer; X.push_back(buffer); }};

#include <fstream>

  /**
   * Helper to check file for additional content
   */
bool fileHasAdditionalContent( std::ifstream& file );


  /**
   * Helper to make windows line endings compatible with linux (modifies the input!)
   */
std::string& trim( std::string& string );


// for model parsing
/**
 * Read a float value from a string.
 *
 * \param[out] f: reference to the float receiving the parsed value (if parsing was successfull)
 * \param s: string to parse
 * @return: 0 if parse was ok, -1 if an error occured.
 */
int parseFloat(float & f,const std::string & s);

/**
 * Read a float value from a string. Uses default value, if _ is found instead of a numeric expression
 *
 * \param[out] f: reference to the float receiving the parsed value (if parsing was successfull)
 * \param s: string to parse
 * \param der: default value
 * @return: 0 if parse was ok, -1 if an error occured.
 */
int parseFloatDefault(float & f, const std::string & s, float def);
/**
 * Read an int value from a string.
 *
 * \param[out] f: reference to the float receiving the parsed value (if parsing was successfull)
 * \param s: string to parse
 * @return: 0 if parse was ok, -1 if an error occured.
 */
int parseInt(int & f,const std::string & s);

/**
 * Read an int value from a string. Uses default value, if _ is found instead of a numeric expression
 *
 * \param[out] f: reference to the float receiving the parsed value (if parsing was successfull)
 * \param s: string to parse
 * \param der: default value
 * @return: 0 if parse was ok, -1 if an error occured.
 */
int parseIntDefault(int & f, const std::string & s, int def);

#define ERROR(E) lastError = E; return E;

#define CHECK_ADD_COMMAND(NAME) if(tokens[0].compare(#NAME) == 0){return NAME(tokens);} if(tokens[0].compare(#NAME "*") == 0){return NAME(tokens);}
#define CHECK_SET_COMMAND(NAME) if(tokens[0].compare(#NAME) == 0){return NAME(tokens);}

#define ISNAMED bool named = (tokens[0].at(tokens[0].length()-1) != '*');
#define UNNAMED bool named = false;

/* TOKEN_NR is the number of tokens excluding the name (if present)
 * i.e. the command "include example.txt" would equal 2
 * and TOKEN_NR is always at least 1 (the command itself) */
#define CHECK_TOKEN_NR(N) if(tokens.size() != (N + (named?1:0))){ERROR(WRONG_NUMBER_OF_ARGUMENTS);}
#define TOKEN_NR_COUNT (tokens.size() - (named?1:0))
#define CHECK_TOKEN_NR_MIN(N) if( ( static_cast<long int>(tokens.size()) ) < ( N + (named?1:0) ) ){ERROR(WRONG_NUMBER_OF_ARGUMENTS);}
#define CHECK_TOKEN_NR_MAX(N) if( ( static_cast<long int>(tokens.size()) ) > ( N + (named?1:0) ) ){ERROR(WRONG_NUMBER_OF_ARGUMENTS);}

/* true if matrix not positive defninit and not zero */
#define CHECK_NOT_POSITIVE_DEFINIT(M) ((M##11 <= 0) || \
	(((M##11 * M##22) - (M##21 * M##12)) <= 0) || \
	(((M##11*M##22*M##33) + (M##12*M##23*M##31) + (M##13*M##21*M##32) - (M##12*M##22*M##31) - (M##12*M##21*M##33) - (M##11*M##23*M##32)) <= 0))\
	&& (!((M##11==0)&&(M##12==0)&&(M##13==0)&&(M##21==0)&&(M##22==0)&&(M##23==0)&&(M##31==0)&&(M##32==0)&&(M##33==0)))
#define CHECK_NOT_SYMMETRIC(M) ((M##12 - M##21) + (M##13 - M##31) + (M##23 - M##32)) != 0

#define LINKNAME (named?tokens[1]:"")

#define PARSE_FLOAT(V,N) float V; if(parseFloat(V,tokens[N+(named?1:0)])){ERROR(TOKEN_PARSE_ERROR);}
#define PARSE_FLOAT_OPT(V,N,D) float V; if(static_cast<long int>(tokens.size()) > N + (named?1:0) ){if(parseFloatDefault(V,tokens[N+(named?1:0)],D)){ERROR(TOKEN_PARSE_ERROR);}}else{V = D;}

#define PARSE_INT(V,N) int V; if(parseInt(V,tokens[N+(named?1:0)])){ERROR(TOKEN_PARSE_ERROR);}
#define PARSE_INT_OPT(V,N,D) int V; if(static_cast<long int>(tokens.size()) > N + (named?1:0) ){if(parseIntDefault(V,tokens[N+(named?1:0)],D)){ERROR(TOKEN_PARSE_ERROR);}}else{V = D;}

#define PARSE_STRING(V,N) std::string V; V = tokens[N+(named?1:0)];

// for delta processing (accuracy)
/**
 * Read the delta value from a fstring (values < 0 are considered not set).
 */
#define PARSE_DELTA(F,DELTA) F >> DELTA;


#endif 