/*
 * Copyright (C) 2015
 * Simulation, Systems Optimization and Robotics Group (SIM)
 * Technische Universitaet Darmstadt
 * Hochschulstr. 10
 * 64289 Darmstadt, Germany
 * www.sim.tu-darmstadt.de
 *
 * This file is part of the MBSlib.
 * All rights are reserved by the copyright holder.
 *
 * MBSlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation in version 3 of the License.
 *
 * The MBSlib is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with MBSlib.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \file mbslib/utility/DeriveOMat.hpp
 * Declaration of mbslib ::DeriveOMat
 */
#ifndef __MBSLIB_DERIVEOMAT_HPP__
#define __MBSLIB_DERIVEOMAT_HPP__

#ifdef USE_ADOLC

#include <mbslib/parameter/ParameterAdapter.hpp>
#include <mbslib/parameter/ParametrizedObjectCollection.hpp>

namespace mbslib {

typedef Eigen::Matrix< double, Eigen::Dynamic, 1 > DomVector;
typedef Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > DomMatrix;

DomVector convert(const DomMatrix & m);

class DeriveOMat {
public:
    DeriveOMat(ParametrizedObjectCollection & poc);

    /**
   * Destructor.
   */
    virtual ~DeriveOMat();

    /**
   * Check, wether we are valid.
   */
    bool isValid() const;

    /**
   * Add an independent variable.
   */
    DeriveOMat & addIndependent(const std::string & oname, const std::string pname);

    /**
   * Add a dependent variable.
   */
    DeriveOMat & addDependent(const std::string & oname, const std::string pname);

    /**
   * Number of independents.
   */
    unsigned int getIndependents() const;

    /**
   * Number of dependents.
   */
    unsigned int getDependents() const;

    /**
   * Start new tape.
   */
    void startTape();

    /**
   * Start new tape.
   */
    void startTape(double const * indep);

    /**
   * Start new tape.
   */
    void startTape(const DomVector & indep);

    /**
   * Finish tape.
   */
    void endTape(double * dep);

    /**
   * Finish tape.
   */
    DomVector endTape();

    /**
   * Evaluate function.
   */
    const double * evaluateFunction(double * indep);

    /**
   * Evaluate function.
   */
    DomVector evaluateFunction(const DomVector & indep);

    /**
   * Evaluate jacobian.
   */
    const double * const * calculateJacobian(double * indep);

    /**
   * Evaluate jacobian.
   */
    DomMatrix calculateJacobian();

    /**
   * Evaluate jacobian.
   */
    DomMatrix calculateJacobian(const DomVector & indep);

protected:
    /// The POC used for this.
    ParametrizedObjectCollection & poc;

    /// Validity of this.
    bool valid;

    /// Tape stated.
    bool tapeStarted;

    /// Tape finished.
    bool tapeFinished;

    /// Independet parameters.
    std::vector< ParameterAdapter > independentParameters;

    /// Dependent parameters.
    std::vector< ParameterAdapter > dependentParameters;

    /// Independents.
    TScalar * independents;

    /// Dependents.
    TScalar * dependents;

    /// Last evaluation.
    double * functionValue;

    /// Last jacobian.
    double ** jacobianValue;

    /// Tag of tape.
    short int tag;

    static size_t tagCounter;
};

} //namespace mbslib

#endif

#endif
