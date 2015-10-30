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
 * @file ErrorLogger.h
 * Definition of class ErrorLogger.
 */

#include "ErrorLogger.h"

/// Definition of static members of ErrorLogger.
std::ostream * ErrorLogger::logstream = 0;
std::ostream * ErrorLogger::warnstream = 0;
std::ostream * ErrorLogger::errorstream = 0;

int ErrorLogger::warnCount = 0;
int ErrorLogger::errorCount = 0;

void ErrorLogger::setLogStream(std::ostream & ls){
  logstream = &ls;
}

void ErrorLogger::disableLogStream(){
  logstream = NULL;
}

void ErrorLogger::setWarnStream(std::ostream & ws){
  warnstream = &ws;
}

void ErrorLogger::setErrorStream(std::ostream & es){
  errorstream = &es;
}