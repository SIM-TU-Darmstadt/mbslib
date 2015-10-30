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
 * \file mbslib/utility/internalTests.cpp
 *
 */
#include <mbslib/utility/internalTests.hpp>

#include <Eigen/LU>

#include <iostream>
using namespace mbslib;

#ifdef USE_ADOLC
bool nearZeroTest(TScalar s) {
#ifdef MBSLIB_DEBUG
    std::cout << "s " << s.value() << "  fabs(s) " << fabs(s.value()) << std::endl;
#endif
    bool rv = (fabs(s.value()) < 1.e-10);
    return rv;
}
#else
bool nearZeroTest(TScalar s) {
    bool rv = (fabs(s) < 1.e-10);
    return rv;
}
#endif

bool mbslib::validInertia(TScalar m, const TMatrix3x3 & I, const std::string & name, const std::string & place, int line) {
    if (m < 0) {
        std::cout << "Mass is < zero in " << name << " in " << place << " line " << line << std::endl;
        return false;
    }
    if (m == 0) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (I(i, j) != 0) {
                    std::cout << "Inertia is not 0 for a mass being zero in " << name << " in " << place << " line " << line << std::endl;
                    return false;
                }
            }
        }
    } else {
        bool posDef = true;
        if (I.determinant() < 0) {
            posDef = false;
        }
        if (I.block< 2, 2 >(0, 0).determinant() < 0) {
            posDef = false;
            ;
        }
        if (I(0, 0) < 0) {
            posDef = false;
        }
        if (!posDef) {
            std::cout << "Inertia is not positive semidefinite in " << name << " in " << place << " line " << line << std::endl;
        }
    }
    return true;
}

bool mbslib::unitVector(const TVector3 & v, const std::string & name, const std::string & place, int line) {
#ifdef MBSLIB_DEBUG
    std::cout << "v: " << v.transpose() << std::endl;
    std::cout << "v.norm() " << v.norm() << std::endl;
    std::cout << "1-v.norm() " << (1 - v.norm()) << std::endl;
#endif
    if (nearZeroTest(1 - v.norm())) {
        return true;
    }
    std::cout << "Denormalized vector " << name << " = (" << v.transpose() << ") has norm !=1, differenced = " << 1 - v.norm() << " in " << place << " line " << line << std::endl;
    std::cout.flush();
    return false;
}

bool mbslib::unitVector(const TVector6 & v, const std::string & name, const std::string & place, int line) {
    if (nearZeroTest(v.norm() - 1)) {
        return true;
    }
    std::cout << "Denormalized vector " << name << " = (" << v.transpose() << ") has norm !=1, differenced = " << 1 - v.norm() << " in " << place << " line " << line << std::endl;
    std::cout.flush();
    return false;
}

bool mbslib::notNearZero(TScalar s, const std::string & name, const std::string & place, int line) {
    if (nearZeroTest(s)) {
        std::cout << "Value " << name << " = " << s << " in " << place << " line " << line << " is near zero." << std::endl;
        return false;
    } else {
        return true;
    }
}
