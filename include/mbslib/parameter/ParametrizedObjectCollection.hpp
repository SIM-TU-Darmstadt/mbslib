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
 * \file mbslib/parameter/ParametrizedObjectCollection.hpp
 * Declaration of mbslib::ParametrizedObjectCollection
 */
#ifndef __PARAMETRIZEDOBJECTCOLLECTION_HPP__
#define __PARAMETRIZEDOBJECTCOLLECTION_HPP__

#include <mbslib/MbslibBaseClass.hpp>
#include <mbslib/parameter/ParametrizedObject.hpp>
#include <vector>
namespace mbslib {

/**
 * \brief Collection of parametrized objects. \todo kommentieren
 */
class ParametrizedObjectCollection : public MbslibBaseClass {
public:
    ParametrizedObjectCollection() {
    }
    ParametrizedObjectCollection(const std::string & name)
        : MbslibBaseClass(name) {
    }
    virtual ~ParametrizedObjectCollection();
    /**
   * \brief Get number of ParametrizedObjects.
   *
   * \return  The parametrized object count.
   */
    size_t getParametrizedObjectCount() const {
        return poc.size();
    }

    /**
   * \brief Get reference to ith ParametrizedObject.
   *
   * \param i Zero-based index of the parameterized object.
   *
   * \return  The i-th parametrized object.
   */
    ParametrizedObject & getParametrizedObject(size_t i) {
        return *poc[i];
    }

    /**
   * \brief Get const reference to ith ParametrizedObject.
   *
   * \param i Zero-based index of the parameterized object.
   *
   * \return  The i-th parametrized object.
   */
    const ParametrizedObject & getParametrizedObject(size_t i) const {
        return *poc[i];
    }

    /**
   * \brief Get pointer to ParametrizedObject with the given name.
   *  
   *  This method will return a NULL pointer if the name is not valid.
   *
   * \param oname The name.
   *
   * \return  null if it fails, else the parametrized object.
   */
    ParametrizedObject * getParametrizedObject(const std::string & oname);

    /**
   * \brief Get const pointer to ParametrizedObject with the given name.
   *  
   *  This method will return a NULL pointer if the name is not valid.
   *
   * \param oname The oname.
   *
   * \return  null if it fails, else the parametrized object.
   */
    const ParametrizedObject * getParametrizedObject(const std::string & oname) const;

    /**
   * \brief Get ParameterAdapter to a given ParametrizedObject.
   *  
   *  This method will return an invalid ParameterAdapter, if either oname or pname are
   *  not valid.
   *
   * \param oname Name or the ParametrizedObject.
   * \param pname Name of the parameter.
   *
   * \return  The parameter adapter.
   */
    ParameterAdapter getParameterAdapter(const std::string & oname, const std::string & pname);

    //protected:

    /**
   * \brief Add a parametrized object to the collection.
   *
   * \param po  The parameterized object.
   */
    void addParametrizedObject(ParametrizedObject & po) {
        poc.push_back(&po);
    }

private:
    /// Vector of objects stored in the collection.
    std::vector< ParametrizedObject * > poc;
};
} //namespace mbslib
#endif
