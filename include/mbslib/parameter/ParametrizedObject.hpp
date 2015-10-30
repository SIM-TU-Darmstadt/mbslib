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
 * \file mbslib/parameter/ParametrizedObject.hpp
 * Declaration of mbslib::ParametrizedObject
 */
#ifndef __PARAMETRIZEDOBJECT_HPP__
#define __PARAMETRIZEDOBJECT_HPP__
#include <mbslib/utility/types.hpp>
#include <mbslib/MbslibBaseClass.hpp>
#include <boost/property_tree/ptree.hpp>
#include <map>
#include <set>
#include <vector>
#include <utility>
#include <functional>

namespace mbslib {
class ParameterAdapter;

/**
 * \brief Parametrized object. \todo kommentieren
 */
class ParametrizedObject : public MbslibBaseClass {
public:
    /**
   * \brief Constructor.
   *
   * \param name  The name of the ParametrizedObject.
   */
    ParametrizedObject(const std::string & name);

    virtual ~ParametrizedObject();

    /**
   * \brief Get number of parameters.
   *
   * \return  The parameter count.
   */
    size_t getParameterCount() const;

    /**
   * \brief Get name of a parameter.
   *
   * \param i Zero-based index of the parameter.
   *
   * \return  The parameter name.
   */
    std::string getParameterName(size_t i) const;

    /**
   * \brief Gets a parameter.
   *
   * \param i Zero-based index of the parameter.
   *
   * \return  The parameter.
   */
    TScalar getParameter(size_t i) const;

    /**
   * \brief Sets a parameter.
   *
   * \param i Zero-based index of the parameter.
   * \param v The value.
   */
    void setParameter(size_t i, TScalar v);

    /**
   * \brief Abstract method to be implemented by subclass.
   *  
   *  This method should not be implemented by hand, but it should be created by using the following
   *  macros: START_PARAMTER_LIST, CONTINUE_PARAMETER_LIST, END_PARAMETER_LIST, PARAMETER,
   *  USE_PARAMETERS_OF.
   *
   * \param op              The operation.
   * \param i               Zero-based index of the parameter.
   * \param [in,out]  value the value.
   *
   * \return \todo kommentieren
   */
    virtual int parameterHandlingMethod(unsigned int op, unsigned int i, void * value) = 0;

    /**
   * \brief Gets a parameter adapter.
   *
   * \param pname The name of the parameter. If this is an invalid name, a invalied ParameterAdapter
   *              will be returned.
   *
   * \return  The parameter adapter.
   */
    ParameterAdapter getParameterAdapter(const std::string pname);
    ParameterAdapter getParameterAdapter(const size_t id);
    /**
   * @brief Gets parameters below the given path.
   * @param path
   * @return
   */
    std::vector< ParameterAdapter > getParameterAdapters(const std::string path);

    std::vector< ParameterAdapter > getParametersOfGroup(const std::string parameterGroupName);
    std::vector< std::string > getParameterGroupNames() const;

protected:
    struct Parameter {
        typedef std::function< TScalar() > GetterFunc_t;
        typedef std::function< void(TScalar) > SetterFunc_t;
        TScalar * value;
        GetterFunc_t valueGetter;
        SetterFunc_t valueSetter;
        std::string name;
        std::set< std::string > parameterAliasSet;
    };
    struct ParameterGroup {
        std::string name;
        //std::vector<Parameter*> parameters;
        //std::vector<ParameterAdapter> parameters;
        std::set< size_t > parameterIDs;
    };

    /**
   * @brief Adds a parameter to the parameter set.
   *
   * Adds a parameter to the parameter set at runtime. The macros: START_PARAMTER_LIST, CONTINUE_PARAMETER_LIST, END_PARAMETER_LIST, PARAMETER,
   *  USE_PARAMETERS_OF add parameters to parameter set at compile time.
   *
   * @param pname
   * @param value
   */
    size_t addParameter(const std::string pname, TScalar & parameter);
    size_t addParameter(const std::string pname, TVectorX & parameterVector, size_t idx);
    size_t addParameter(const std::string pname, TMatrixX & parameterMatrix, size_t xidx, size_t yidx);
    size_t addParameter(const std::string pname, Parameter::GetterFunc_t getter = Parameter::GetterFunc_t(), Parameter::SetterFunc_t setter = Parameter::SetterFunc_t());
    std::vector< size_t > addParameters(const std::string pname_prefix, TVectorX & parameterVector, size_t countOffset = 0);
    std::vector< size_t > addParameters(const std::string pname_prefix, TMatrixX & parameterMatrix, size_t countOffsetRow = 0, size_t countOffsetCol = 0, const std::string & colRowSeparator = ".");
    bool addParameterAlias(const std::string pname, const std::string & path);
    bool addParameterAlias(const size_t i, const std::string & path);
    void removeParameterAlias(const size_t i, const std::string & path);

    bool addParameterToGroup(const std::string parameterGroupName, const std::string pname);
    bool addParameterToGroup(const std::string parameterGroupName, const size_t paramid);
    bool addParametersToGroup(const std::string parameterGroupName, const std::vector< std::string > pnames);
    bool addParametersToGroup(const std::string parameterGroupName, const std::vector< size_t > paramIDs);

    int parameterHandling(unsigned int op, unsigned int i, void * value);

    virtual std::vector< Parameter * > & getParameterVector();

protected:
    Parameter * getParameterByName(const std::string pname);
    std::vector< Parameter * > parameters;
    boost::property_tree::ptree parameterTree;
    std::map< std::string, Parameter * > parameterMap;

    std::map< std::string, ParameterGroup > parameterGroupMap;

}; // class ParametrizedObject

#define START_PARAMETER_LIST                                                             \
public:                                                                                  \
    virtual int parameterHandlingMethod(unsigned int op, unsigned int i, void * value) { \
        const int superParams = 0;                                                       \
        const int anchorline = __LINE__;                                                 \
        const int dynParameterCount = parameters.size();                                 \
        switch (op) {                                                                    \
        case 0:                                                                          \
        case 1:                                                                          \
        case 2:                                                                          \
            switch (i + anchorline + 1) {

#define CONTINUE_PARAMETER_LIST(SUPERCLASS)                                                                                   \
public:                                                                                                                       \
    virtual int parameterHandlingMethod(unsigned int op, unsigned int i, void * value) {                                      \
        const int superParams = SUPERCLASS::parameterHandlingMethod(3, 0, 0);                                                 \
        if ((superParams >= 0) && (i < (unsigned int)superParams) && (op < 3)) {                                              \
            return SUPERCLASS::parameterHandlingMethod(op, i, value);                                                         \
        }                                                                                                                     \
        const int anchorline = __LINE__;                                                                                      \
        const int dynParameterCount = ((&SUPERCLASS::getParameterVector() == &getParameterVector()) ? 0 : parameters.size()); \
        switch (op) {                                                                                                         \
        case 0:                                                                                                               \
        case 1:                                                                                                               \
        case 2:                                                                                                               \
            switch (i + anchorline + 1 - superParams) {

#define PARAMETER(NAME, GET, SET)                                  \
    case __LINE__:                                                 \
        if (op == 0) {                                             \
            static_cast< std::string * >(value)->operator=(#NAME); \
            return 0;                                              \
        }                                                          \
        if (op == 1) {                                             \
            *static_cast< mbslib::TScalar * >(value) = GET;        \
            return 0;                                              \
        }                                                          \
        if (op == 2) {                                             \
            SET = *static_cast< mbslib::TScalar * >(value);        \
            return 0;                                              \
        }

#define END_PARAMETER_LIST                                                    \
    default: {                                                                \
        const int ctParameterCount = __LINE__ - anchorline - 1 + superParams; \
        const int parameterCount = dynParameterCount + ctParameterCount;      \
        if (i >= ctParameterCount && i < parameterCount) {                    \
            return parameterHandling(op, i - ctParameterCount, value);        \
        }                                                                     \
        return -1;                                                            \
    }                                                                         \
        }                                                                     \
    case 3:                                                                   \
        return __LINE__ - anchorline - 1 + superParams + dynParameterCount;   \
    case 4:                                                                   \
        return __LINE__ - anchorline - 1 + superParams;                       \
    case 5:                                                                   \
        return dynParameterCount;                                             \
    default:                                                                  \
        return -1;                                                            \
        }                                                                     \
        return -1;                                                            \
        }

#define USE_PARAMETERS_OF(X)                                                             \
    virtual int parameterHandlingMethod(unsigned int op, unsigned int i, void * value) { \
        return X->parameterHandlingMethod(op, i, value);                                 \
    }

#define EMPTY_PARAMETER_LIST                                                                         \
    virtual int parameterHandlingMethod(unsigned int /*op*/, unsigned int /*i*/, void * /*value*/) { \
        return 0;                                                                                    \
    }
}
#endif
