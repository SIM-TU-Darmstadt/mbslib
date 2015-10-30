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

#ifndef __MBSLIB__MBSLIB_BASE_CLASS_HPP__
#define __MBSLIB__MBSLIB_BASE_CLASS_HPP__

#include <string>
namespace mbslib {
class MbslibBaseClass {
private:
    const std::string * _name;

public:
    virtual void toString(std::ostream & out) const;
    MbslibBaseClass();
    MbslibBaseClass(const std::string & name);
    virtual ~MbslibBaseClass();

    /**
    * \brief Sets a name.
    *
    * \param name  The name.
    */
    void setName(const std::string & name);

    /**
    * \brief Gets the name.
    *
    * \return  null if it fails, else the name.
    */
    const std::string & getName() const;
};
}

#endif