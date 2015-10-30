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
* \file mbslib/parameter/ParameterAdapter.hpp
* Declaration of mbslib::ParameterAdapter
*/
#ifndef __PARAMETERADAPTER_HPP__
#define __PARAMETERADAPTER_HPP__

#include <mbslib/parameter/ParametrizedObject.hpp>
#include <mbslib/utility/types.hpp>

#include <assert.h>
namespace mbslib {

/**
  * \brief Parameter adapter. \todo kommentieren
  */
class ParameterAdapter {
    friend class ParametrizedObject;

public:
    /**
    * \brief Default constructor.
    */
    ParameterAdapter();
    virtual ~ParameterAdapter();

    /**
    * \brief Constructor. \todo kommentieren
    *
    * \param o The parameterized object.
    * \param p Parameter ID.
    */
    ParameterAdapter(ParametrizedObject & o, size_t p);

    /**
    * \brief Check for validity of ParameterAdapter.
    *  
    *  If this return false, the ParameterAdapter does not access an existing parameter.
    *
    * \return  true if valid, false if not.
    */
    bool isValid() const;

    /**
    * \brief Assignment of a value to the parameter.
    *
    * \param value The value.
    *
    * \return  A shallow copy of this object.
    */
    TScalar operator=(TScalar value);

    /**
    * \brief Conversion operator to scalar type.
    */
    operator TScalar();

    std::string getName() const;
    size_t getID() const;

protected:
    /// The object.
    ParametrizedObject * object;

    /// The parameter id.
    size_t paramid;

}; //class ParameterAdapter
}
#endif
