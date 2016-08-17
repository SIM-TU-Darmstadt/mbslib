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

#include <algorithm>
#include <mbslib/MbslibBaseClass.hpp>
#include <ostream>

using namespace mbslib;
using namespace std;

void MbslibBaseClass::toString(ostream & out) const {
    out << "<object>" << endl;
    out << "  <name>" << this->getName() << "</name>" << endl;
    out << "</object>" << endl;
}

MbslibBaseClass::MbslibBaseClass(const std::string & name) {
    setName(name);
}

MbslibBaseClass::MbslibBaseClass()
    : _name(0) {
}
MbslibBaseClass::~MbslibBaseClass() {
    delete _name;
}
void MbslibBaseClass::setName(const std::string & name) {
    if (!name.compare("")) {
        _name = 0;
        return;
    }
    _name = new std::string(name);
}

const std::string & MbslibBaseClass::getName() const {
    static const std::string defaultName = "<NONAME>";
    if (!_name) {
        return defaultName;
    }
    return *_name;
}
