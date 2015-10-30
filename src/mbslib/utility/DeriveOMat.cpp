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
 * \file mbslib/utility/DeriveOMat.cpp
 * Definition of mbslib ::DeriveOMat
 */

#ifdef USE_ADOLC

#include <mbslib/utility/DeriveOMat.hpp>

using namespace mbslib;

namespace mbslib {
DomVector convert(const DomMatrix & m) {
    DomVector v;
    v.resize(m.cols() * m.rows());
    for (size_t col = 0, vidx = 0; col < m.cols(); ++col) {
        for (size_t row = 0; row < m.rows(); ++row) {
            v[vidx++] = m(row, col);
        }
    }
    return v;
}
}
size_t DeriveOMat::tagCounter = 0;

DeriveOMat::DeriveOMat(ParametrizedObjectCollection & poc)
    : poc(poc)
    , valid(true)
    , tapeStarted(false)
    , tapeFinished(false)
    , independents(NULL)
    , dependents(NULL)
    , functionValue(NULL)
    , jacobianValue(NULL)
    , tag(tagCounter++) {
    ///TODO: add destructor which cleans up
    ///TODO: get a free tape number
}

DeriveOMat::~DeriveOMat() {
    if (independents) {
        delete[] independents;
    }
    if (dependents) {
        delete[] dependents;
    }
    if (functionValue) {
        delete[] functionValue;
    }
    if (jacobianValue) {
        for (unsigned int i = 0; i < getDependents(); i++) {
            delete jacobianValue[i];
        }
        delete jacobianValue;
    }
}

DeriveOMat & DeriveOMat::addIndependent(const std::string & oname, const std::string pname) {
    assert(!tapeStarted);
    ParameterAdapter pa = poc.getParameterAdapter(oname, pname);
    if (!pa.isValid()) {
        std::cout << "DeriveOMat::addIndependent -- failed for " << oname << " - " << pname << std::endl;
    }
    valid = (valid && pa.isValid());
    independentParameters.push_back(pa);
    return *this;
}

DeriveOMat & DeriveOMat::addDependent(const std::string & oname, const std::string pname) {
    assert(!tapeStarted);
    ParameterAdapter pa = poc.getParameterAdapter(oname, pname);
    if (!pa.isValid()) {
        std::cout << "DeriveOMat::addDependent -- failed for " << oname << " - " << pname << std::endl;
    }
    valid = (valid && pa.isValid());
    dependentParameters.push_back(pa);
    return *this;
}

void DeriveOMat::startTape() {
    assert(valid);
    assert(!tapeStarted);
    tapeStarted = true;
    tapeFinished = false;
    if (independents) {
        delete[] independents;
    }
    independents = new TScalar[independentParameters.size()];

    std::vector< double > indep(independentParameters.size());
    for (size_t i = 0, size = independentParameters.size(); i < size; ++i) {
        indep[i] = ((TScalar)independentParameters[i]).getValue();
    }

    trace_on(tag);

    for (size_t i = 0, size = independentParameters.size(); i < size; ++i) {
        independents[i] <<= indep[i];
        independentParameters[i] = independents[i];
    }
}

void DeriveOMat::startTape(double const * indep) {
    assert(valid);
    assert(!tapeStarted);
    tapeStarted = true;
    tapeFinished = false;
    if (independents) {
        delete[] independents;
    }
    independents = new TScalar[independentParameters.size()];

    trace_on(tag);

    for (unsigned int i = 0; i < independentParameters.size(); i++) {
        independents[i] <<= indep[i];
        independentParameters[i] = independents[i];
    }
}

double * eigen2pointer(const DomVector & v) {
    double * p = new double[v.size()];
    assert(p);
    for (int i = 0; i < v.size(); i++) {
        //  std::cout << i;
        p[i] = v(i);
    }
    return p;
}

DomVector pointer2eigen(const double * p, int size) {
    DomVector v(size);
    for (int i = 0; i < size; i++) {
        v(i) = p[i];
    }
    return v;
}

DomMatrix pointer2eigen(const double * const * p, int r, int c) {
    DomMatrix m(r, c);
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            m(i, j) = p[i][j];
        }
    }
    return m;
}

void DeriveOMat::startTape(const DomVector & indep) {
    double * i = eigen2pointer(indep);
    startTape(i);
    delete[] i; //i must be kept as long as the tape exists, otherways deleting independents will crash
}

void DeriveOMat::endTape(double * dep) {
    assert(valid);
    assert(tapeStarted);
    tapeFinished = true;
    tapeStarted = false;
    if (dependents) {
        delete[] dependents;
    }
    dependents = new TScalar[dependentParameters.size()];

    for (unsigned int i = 0; i < dependentParameters.size(); i++) {
        dependents[i] = dependentParameters[i];
        dependents[i] >>= dep[i];
    }

    trace_off(tag);
}

DomVector DeriveOMat::endTape() {
    std::vector< double > dep(dependentParameters.size());
    endTape(dep.data());
    DomVector v = Eigen::Map< DomVector >(dep.data(), dependentParameters.size());
    return v;
}

const double * DeriveOMat::evaluateFunction(double * indep) {
    assert(tapeFinished);

    if (functionValue) {
        delete[] functionValue;
    }
    functionValue = new double[getDependents()];

    ::function(tag, getDependents(), getIndependents(), indep, functionValue);

    return functionValue;
}

DomVector DeriveOMat::evaluateFunction(const DomVector & indep) {
    double * i = eigen2pointer(indep);
    const double * v = evaluateFunction(i);
    delete[] i;
    return pointer2eigen(v, getDependents());
}

const double * const * DeriveOMat::calculateJacobian(double * indep) {
    assert(tapeFinished);

    if (jacobianValue) {
        for (unsigned int i = 0; i < getDependents(); i++) {
            delete jacobianValue[i];
        }
        delete jacobianValue;
    }
    jacobianValue = new double *[getDependents()];
    for (unsigned int i = 0; i < getDependents(); i++) {
        jacobianValue[i] = new double[getIndependents()];
    }

    ::jacobian(tag, getDependents(), getIndependents(), indep, jacobianValue);

    return jacobianValue;
}

DomMatrix DeriveOMat::calculateJacobian() {
    DomVector indep;
    indep.resize(independentParameters.size());
    for (size_t i = 0, size = independentParameters.size(); i < size; ++i) {
        indep[i] = ((TScalar)independentParameters[i]).getValue();
    }
    return calculateJacobian(indep);
}

DomMatrix DeriveOMat::calculateJacobian(const DomVector & indep) {
    assert(indep.size() == getIndependents());
    double * i = eigen2pointer(indep);
    const double * const * j = calculateJacobian(i);
    delete[] i;
    return pointer2eigen(j, getDependents(), getIndependents());
}

/**
 * Check, wether we are valid.
 */
bool DeriveOMat::isValid() const {
    return valid;
}

/**
 * Number of independents.
 */
unsigned int DeriveOMat::getIndependents() const {
    return independentParameters.size();
}

/**
 * Number of dependents.
 */
unsigned int DeriveOMat::getDependents() const {
    return dependentParameters.size();
}
#endif // USE_ADOLC
