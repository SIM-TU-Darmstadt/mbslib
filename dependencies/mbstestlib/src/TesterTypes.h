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
 * @file TesterTypes.h
 * Declaration of basic types used within the tester.
 */

#ifndef TESTERTYPES_H_
#define TESTERTYPES_H_

#include <math.h>
#include <iostream>

/// The type to represent real numbers.
//typedef float TesterScalar;
typedef double TesterScalar; // double necessary for eigen (mbslib)

/// The type to represent integer numbers.
typedef int TesterInt;

/// The type to represent the time.
typedef TesterScalar TesterTime;

/// Maximum value for time.
const TesterScalar MAX_TIME = 3.402823466e+38f;

/// The type to represent twodimensional real vectors.
//typedef TesterScalar TesterVector2[2];
typedef TesterScalar TesterVector2[2];

/// The type to represent threedimensional real vectors.
class TesterVector3{
public:
  /**
   * Default Constructor (creates 0,0,0-vec)
   */
  TesterVector3(){for (int i=0;i<3;i++) vectorValues[i]=0;};

  /**
   * Constructor for arbitrary vectors
   */
  TesterVector3(TesterScalar x, TesterScalar y, TesterScalar z){vectorValues[0]=x;vectorValues[1]=y;vectorValues[2]=z;};

  /**
   * Constructor for arbitrary vectors
   */
  TesterVector3(TesterScalar valueArray[3]){for (int i=0;i<3;i++) vectorValues[i]=valueArray[i];};

  /**
   * Destructor
   */
  ~TesterVector3(){};

	bool isZero(){
		return (vectorValues[0]==0 && vectorValues[1]==0 && vectorValues[2]==0);
	};

	TesterScalar length(){
		return sqrt((vectorValues[0]*vectorValues[0])+(vectorValues[1]*vectorValues[1])+(vectorValues[2]*vectorValues[2]));
	};

	void rescale2length1(){
		vectorValues[0]/=this->length();
		vectorValues[1]/=this->length();
		vectorValues[2]/=this->length();
	};

  TesterScalar vectorValues[3];
};
/// toString-like function
std::ostream& operator<<(std::ostream& out, const TesterVector3& val);


/// The type to represent real 3x3-matrices.
class TesterMatrix3x3{
public:
  /**
   * Default Constructor (creates 0,0,0;0,0,0;0,0,0-matrix)
   */
  TesterMatrix3x3(){
	  for (int i=0;i<3;i++) 
		  for (int j=0;j<3;j++)
		     matrixValues[i][j]=0;
  };

  /**
   * Constructor for arbitrary matrices (format: {row1,row2,row3})
   */
  TesterMatrix3x3(TesterScalar valueArray[9]){
	  for (int i=0;i<3;i++)
		  for (int j=0;j<3;j++)
			  matrixValues[i][j]=valueArray[i*3+j];
  };

  /**
   * Destructor
   */
  ~TesterMatrix3x3(){};

	TesterMatrix3x3 setIdentity(){
		setZero();
		matrixValues[0][0]=1;
		matrixValues[1][1]=1;
		matrixValues[2][2]=1;
		return *this;
	}

	TesterMatrix3x3 setZero(){
		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++)
				matrixValues[i][j]=0;
		return *this;
	}

	bool isEqual(const TesterMatrix3x3 & other)const{
		bool equality = true;
		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++)
				if (matrixValues[i][j] != other.matrixValues[i][j]) equality = false;
		return equality;
	}

  TesterScalar matrixValues[3][3];
};
/// toString-like function
std::ostream& operator<<(std::ostream& out, const TesterMatrix3x3& val);

#endif /* TESTERTYPES_H_ */
