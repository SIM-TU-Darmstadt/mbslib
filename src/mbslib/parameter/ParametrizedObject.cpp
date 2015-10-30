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
 * \file mbslib/parameter/ParametrizedObject.cpp
 * Definition of mbslib::ParametrizedObject
 */

#include <mbslib/parameter/ParametrizedObject.hpp>
#include <iostream>
#include <mbslib/parameter/ParameterAdapter.hpp>
#include <boost/lexical_cast.hpp>

using namespace mbslib;
ParametrizedObject::ParametrizedObject(const std::string & name)
    : MbslibBaseClass(name) {
}

ParametrizedObject::~ParametrizedObject() {
    for (Parameter * p : parameters) {
        delete p;
    }
    parameters.clear();
}

size_t ParametrizedObject::getParameterCount() const {
    return const_cast< ParametrizedObject * >(this)->parameterHandlingMethod(3, 0, NULL);
}

std::string ParametrizedObject::getParameterName(size_t i) const {
    std::string s;
    const_cast< ParametrizedObject * >(this)->parameterHandlingMethod(0, i, &s);
    return s;
}

TScalar ParametrizedObject::getParameter(size_t i) const {
    TScalar s;
    const_cast< ParametrizedObject * >(this)->parameterHandlingMethod(1, i, &s);
    return s;
}

void ParametrizedObject::setParameter(size_t i, TScalar v) {
    parameterHandlingMethod(2, i, &v);
}

ParameterAdapter ParametrizedObject::getParameterAdapter(const std::string pname) {
    for (size_t i = 0; i < getParameterCount(); ++i) {
        if (getParameterName(i) == pname) {
            return ParameterAdapter(*this, i);
        }
    }
    {
        boost::optional< std::uintptr_t > pptr = parameterTree.get_optional< std::uintptr_t >(pname);
        if (pptr.is_initialized()) {
            Parameter * p_it = reinterpret_cast< Parameter * >(pptr.get());
            for (size_t i = 0; i < parameters.size(); ++i) {
                Parameter * p = parameters.at(i);
                if (p == p_it) {
                    return ParameterAdapter(*this, i + getParameterCount() - parameters.size());
                }
            }
        }
        //      auto it = parameterTree.find(pname);
        //      if(it != parameterTree.not_found()) {
        //          Parameter* p_it = reinterpret_cast<Parameter*>(it->second.get_value<std::uintptr_t>());
        //          for(size_t i = 0; i < parameters.size(); ++i) {
        //              Parameter& p = parameters.at(i);
        //              if(&p == p_it) {
        //                  return ParameterAdapter(*this, i+getParameterCount()-parameters.size());
        //              }

        //          }
        //      }
    }
    std::cout << "ParametrizedObject::getParameterAdapter -- unknown parameter " << pname << std::endl;
    return ParameterAdapter();
}

ParameterAdapter ParametrizedObject::getParameterAdapter(const size_t id) {
    if (id < getParameterCount()) {
        return ParameterAdapter(*this, id);
    }
    return ParameterAdapter();
}

size_t ParametrizedObject::addParameter(const std::string pname, TScalar & parameter) {
    size_t pid = getParameterCount();
    Parameter * p = new Parameter({&parameter, Parameter::GetterFunc_t(), Parameter::SetterFunc_t(), pname});
    p->parameterAliasSet.insert(pname);
    parameters.push_back(p);
    parameterTree.add(pname, reinterpret_cast< std::uintptr_t >(p));
    parameterMap[pname] = p;
    return pid;
}

size_t ParametrizedObject::addParameter(const std::string pname, TVectorX & parameterVector, size_t idx) {
    Parameter::GetterFunc_t getter = [idx, &parameterVector]() { return parameterVector[idx]; };
    Parameter::SetterFunc_t setter = [idx, &parameterVector](TScalar parameter) { parameterVector[idx] = parameter; };
    return addParameter(pname, getter, setter);
}

size_t ParametrizedObject::addParameter(const std::string pname, TMatrixX & parameterMatrix, size_t xidx, size_t yidx) {
    Parameter::GetterFunc_t getter = [xidx, yidx, &parameterMatrix]() { return parameterMatrix(xidx, yidx); };
    Parameter::SetterFunc_t setter = [xidx, yidx, &parameterMatrix](TScalar parameter) { parameterMatrix(xidx, yidx) = parameter; };
    return addParameter(pname, getter, setter);
}
size_t ParametrizedObject::addParameter(const std::string pname, Parameter::GetterFunc_t getter, Parameter::SetterFunc_t setter) {
    size_t pid = getParameterCount();
    Parameter * p = new Parameter({nullptr, getter, setter, pname});
    p->parameterAliasSet.insert(pname);
    parameters.push_back(p);
    parameterTree.add(pname, reinterpret_cast< std::uintptr_t >(p));
    parameterMap[pname] = p;
    return pid;
}
std::vector< size_t > ParametrizedObject::addParameters(const std::string pname_prefix, TVectorX & parameterVector, size_t countOffset) {
    std::vector< size_t > pids;
    for (size_t i = 0; i < parameterVector.size(); ++i) {
        std::stringstream ss;
        ss << pname_prefix;
        ss << (countOffset + i);
        size_t pid = addParameter(ss.str(), parameterVector, i);
        pids.push_back(pid);
    }
    return pids;
}
std::vector< size_t > ParametrizedObject::addParameters(const std::string pname_prefix, TMatrixX & parameterMatrix, size_t countOffsetRow, size_t countOffsetCol, const std::string & colRowSeparator) {
    std::vector< size_t > pids;
    for (size_t row = 0,
                rowCount = parameterMatrix.rows(),
                colCount = parameterMatrix.cols();
         row < rowCount; ++row) {
        for (size_t col = 0; col < colCount; ++col) {
            std::stringstream ss;
            ss << pname_prefix;
            ss << (countOffsetRow + row) << colRowSeparator << (countOffsetCol + col);
            size_t pid = addParameter(ss.str(), parameterMatrix, row, col);
            pids.push_back(pid);
        }
    }
    return pids;
}

bool ParametrizedObject::addParameterAlias(const std::string pname, const std::string & path) {
    boost::optional< std::uintptr_t > pptr = parameterTree.get_optional< std::uintptr_t >(pname);
    boost::optional< std::uintptr_t > pptrPath = parameterTree.get_optional< std::uintptr_t >(path);
    if (pptrPath.is_initialized()) {
        return false;
    }
    if (pptr.is_initialized()) {
        Parameter * p_it = reinterpret_cast< Parameter * >(pptr.get());
        if (p_it->parameterAliasSet.count(path) > 0) {
            return false;
        }
        p_it->parameterAliasSet.insert(path);
        parameterTree.add(path, reinterpret_cast< std::uintptr_t >(p_it));
        parameterMap[path] = p_it;
        return true;
    }
    return false;
}

bool ParametrizedObject::addParameterAlias(const size_t i, const std::string & path) {
    if (i < getParameterCount() - parameters.size()) {
        /// not yet supported for compile time parameters
        return false;
    }
    size_t parameterID = i - (getParameterCount() - parameters.size());
    Parameter * p = parameters.at(parameterID);
    if (path == p->name) {
        /// path already existing
        return false;
    }
    {
        auto it = p->parameterAliasSet.find(path);
        if (it != p->parameterAliasSet.end()) {
            /// path already set for parameter
            return false;
        }
    }
    {
        auto it = parameterTree.find(path);
        if (it != parameterTree.not_found()) {
            /// path already existing for some other parameter
            return false;
        }
    }
    p->parameterAliasSet.insert(path);
    parameterTree.put(path, reinterpret_cast< std::uintptr_t >(p));
    parameterMap[path] = p;

    /*boost::optional<std::uintptr_t> opt = parameterTree.get_optional<std::uintptr_t>(path);
    if(opt.is_initialized()) {
        std::cout << "path: " << path << " value: " << opt.get() << std::endl;
        //std::cout << "bla.G: " << opt.get() << std::endl;
    } else {
        throw std::runtime_error("");
    }*/

    return true;
}

void ParametrizedObject::removeParameterAlias(const size_t i, const std::string & path) {
    if (i < getParameterCount() - parameters.size()) {
        /// not yet supported for compile time parameters
        return;
    }
    size_t parameterID = i - (getParameterCount() - parameters.size());
    Parameter * p = parameters.at(parameterID);
    if (path == p->name) {
        /// do not allow to remove parameter itself from path
        return;
    }
    {
        auto it = p->parameterAliasSet.find(path);
        if (it == p->parameterAliasSet.end()) {
            return;
        }
        p->parameterAliasSet.erase(it);
    }
    {
        auto it = parameterTree.find(path);
        if (it == parameterTree.not_found()) {
            return;
        }
        parameterTree.erase(parameterTree.to_iterator(it));
    }
    parameterMap.erase(path);
}
int ParametrizedObject::parameterHandling(unsigned int op, unsigned int i, void * value) {
    if (i >= parameters.size()) {
        return -1;
    }
    Parameter * p = parameters.at(i);
    switch (op) {
    case 0:
        static_cast< std::string * >(value)->operator=(p->name);
        return 0;
    case 1: {
        if (p->value) {
            *static_cast< mbslib::TScalar * >(value) = *p->value;
        } else if (p->valueGetter) {
            *static_cast< mbslib::TScalar * >(value) = p->valueGetter();
        } else {
            return -1;
        }
        return 0;
    }
    case 2: {
        if (p->value) {
            *p->value = *static_cast< mbslib::TScalar * >(value);
        } else if (p->valueSetter) {
            p->valueSetter(*static_cast< mbslib::TScalar * >(value));
        } else {
            return -1;
        }
        return 0;
    }
    default:
        return -1;
    }
    return 0;
}

std::vector< ParametrizedObject::Parameter * > & ParametrizedObject::getParameterVector() {
    return parameters;
}

ParametrizedObject::Parameter * ParametrizedObject::getParameterByName(const std::string pname) {
    Parameter * retval = nullptr;
    for (Parameter * p : parameters) {
        if (p->name == pname) {
            retval = p;
            break;
        }
    }
    return retval;
}

bool ParametrizedObject::addParameterToGroup(const std::string parameterGroupName, const std::string pname) {
    ParameterAdapter pa = getParameterAdapter(pname);
    if (!pa.isValid()) {
        return false;
    }
    //Parameter* p = getParameter(pname);
    //if(p == nullptr) {
    //    return false;
    //}

    auto it = parameterGroupMap.find(parameterGroupName);
    if (it == parameterGroupMap.end()) {
        parameterGroupMap[parameterGroupName] = ParameterGroup();
        parameterGroupMap[parameterGroupName].name = parameterGroupName;
    }

    ParameterGroup & pgroup = parameterGroupMap[parameterGroupName];

    if (pgroup.parameterIDs.find(pa.paramid) != pgroup.parameterIDs.end()) {
        return true;
    }

    //pgroup.parameters.push_back(pa);
    pgroup.parameterIDs.insert(pa.paramid);
    return true;
}
bool ParametrizedObject::addParameterToGroup(const std::string parameterGroupName, const size_t paramid) {
    if (paramid >= getParameterCount()) {
        return false;
    }

    auto it = parameterGroupMap.find(parameterGroupName);
    if (it == parameterGroupMap.end()) {
        parameterGroupMap[parameterGroupName] = ParameterGroup();
        parameterGroupMap[parameterGroupName].name = parameterGroupName;
    }

    ParameterGroup & pgroup = parameterGroupMap[parameterGroupName];

    if (pgroup.parameterIDs.find(paramid) != pgroup.parameterIDs.end()) {
        return true;
    }

    //pgroup.parameters.push_back(pa);
    pgroup.parameterIDs.insert(paramid);
    return true;
}

bool ParametrizedObject::addParametersToGroup(const std::string parameterGroupName, const std::vector< std::string > pnames) {
    bool retval = true;
    for (std::string pname : pnames) {
        retval = retval && addParameterToGroup(parameterGroupName, pname);
    }
    return retval;
}

bool ParametrizedObject::addParametersToGroup(const std::string parameterGroupName, const std::vector< size_t > paramIDs) {
    bool retval = true;
    for (size_t pID : paramIDs) {
        retval = retval && addParameterToGroup(parameterGroupName, pID);
    }
    return retval;
}

std::vector< ParameterAdapter > ParametrizedObject::getParametersOfGroup(const std::string parameterGroupName) {
    auto it = parameterGroupMap.find(parameterGroupName);
    if (it == parameterGroupMap.end()) {
        return std::vector< ParameterAdapter >();
    }

    ParameterGroup & pgroup = parameterGroupMap[parameterGroupName];

    std::vector< ParameterAdapter > parameters;
    parameters.reserve(pgroup.parameterIDs.size());
    for (size_t pid : pgroup.parameterIDs) {
        parameters.push_back(getParameterAdapter(getParameterName(pid)));
    }
    return parameters;
    //return pgroup.parameters;
}

std::vector< std::string > ParametrizedObject::getParameterGroupNames() const {
    std::vector< std::string > names;
    names.reserve(parameterGroupMap.size());
    for (auto imap : parameterGroupMap) {
        names.push_back(imap.first);
    }
    return names;
}
