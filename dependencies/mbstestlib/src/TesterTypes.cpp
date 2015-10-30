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
 * @file TesterTypes.cpp
 * Definition of basic types used within the tester.
 */

#include "TesterTypes.h"

std::ostream& operator<<(std::ostream& out, const TesterVector3& val){
	return out << val.vectorValues[0] << " " << val.vectorValues[1] << " " << val.vectorValues[2];
};

std::ostream& operator<<(std::ostream& out, const TesterMatrix3x3& val)
{
	return out<< val.matrixValues[0][0] << " " << val.matrixValues[0][1] << " " << val.matrixValues[0][2] << "  "
		<< val.matrixValues[1][0] << " " << val.matrixValues[1][1] << " " << val.matrixValues[1][2] << "  "
		<< val.matrixValues[2][0] << " " << val.matrixValues[2][1] << " " << val.matrixValues[2][2];
};
