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
 * Declaration of class ErrorLogger and macros to acces the logger.
 */

#ifndef __ErrorLogger_h__
#define __ErrorLogger_h__

#include <ostream>

#define LOGOUT(X) {if(ErrorLogger::isLogStreamValid()){ErrorLogger::getLogStream() << X << std::endl;}}
//#define WARNOUT(X) {if(ErrorLogger::isLogStreamValid()){ErrorLogger::getLogStream() << "_warnout: " << X << std::endl;} if(ErrorLogger::isWarnStreamValid()){ErrorLogger::getWarnStream() << "_warnout: " << X << std::endl;} ErrorLogger::incWarnCount();}
#define WARNOUT(X) {if(ErrorLogger::isWarnStreamValid()){ErrorLogger::getWarnStream() << "_warnout: " << X << std::endl;} ErrorLogger::incWarnCount();}
//#define ERROUT(X) {if(ErrorLogger::isLogStreamValid()){ErrorLogger::getLogStream() << "ERROUT__: " << X << std::endl;} if(ErrorLogger::isWarnStreamValid()){ErrorLogger::getWarnStream() << "ERROUT__: " << X << std::endl;} if(ErrorLogger::isErrorStreamValid()){ErrorLogger::getErrorStream() << "ERROUT__: " << X << std::endl;} ErrorLogger::incErrorCount();}
#define ERROUT(X) {if(ErrorLogger::isErrorStreamValid()){ErrorLogger::getErrorStream() << "ERROUT__: " << X << std::endl;} ErrorLogger::incErrorCount();}
#define ALLOUT(X) {if(ErrorLogger::isLogStreamValid()){ErrorLogger::getLogStream() << "          " << X << std::endl;} if(ErrorLogger::isWarnStreamValid()){ErrorLogger::getLogStream() << "          " << X << std::endl;} if(ErrorLogger::isErrorStreamValid()){ErrorLogger::getErrorStream() << "          " << X << std::endl;}}


/**
 * A class with static members to write log messages to streams.
 **/
class ErrorLogger{
public:
  /**
   * Set log stream.
   *
   * \param ls The logstream.
   */
  static void setLogStream(std::ostream & ls);

  /**
   * Disable log stream.
   */
  static void disableLogStream();
  
  /**
   * Check if stream for log messages is valid.
   */
  static bool isLogStreamValid(){return logstream != NULL;}
  
  /**
   * Get stream to write log messages to.
   */
  static std::ostream & getLogStream(){return *logstream;}
  
  /**
   * Set warn stream.
   *
   * \param ws The warnstream.
   */
  static void setWarnStream(std::ostream & ws);

  /**
   * Check if stream for warn messages is valid.
   */
  static bool isWarnStreamValid(){return warnstream != NULL;}
  
  /**
   * Get stream to write warn messages to.
   */
  static std::ostream & getWarnStream(){return *warnstream;}

  /**
   * Get error count.
   */
  static int getWarnCount(){return warnCount;}
  
  /**
   * Increment warncount.
   */
  static void incWarnCount(){warnCount++;}
  
  /**
   * Set error stream.
   *
   * \param es The errorstream.
   */
  static void setErrorStream(std::ostream & es);

  /**
   * Check if stream for error messages is valid.
   */
  static bool isErrorStreamValid(){return errorstream != NULL;}
  
  /**
   * Get stream to write error messages to.
   */
  static std::ostream & getErrorStream(){return *errorstream;}
  
  /**
   * Increment errorcount.
   */
  static void incErrorCount(){errorCount++;}

  /**
   * Get error count.
   */
  static int getErrorCount(){return errorCount;}
  
private:
  /// The logstream.
  static std::ostream * logstream;
  
  /// The warnstream.
  static std::ostream * warnstream;
  
  /// The errorstream.
  static std::ostream * errorstream;
  
  /// The warncount.
  static int warnCount;
  
  /// The errorcount.
  static int errorCount;
}; // class ErrorLogger


#endif
