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
 * \file mbslib/elements/spring/Spring3D.cpp
 * Definition of mbslib ::Spring3D
 */

#include <fstream>
#include <iostream>
#include <mbslib/elements/spring/Spring3D.hpp>

namespace mbslib {

class SpringSegment {
public:
    /**
   * Constructor.
   */
    SpringSegment(Endpoint & end1, Endpoint & end2)
        : end1(end1)
        , end2(end2)
        , length(0)
        , dLength(0) {
        direction.setZero();
        vRel.setZero();
    }

    virtual ~SpringSegment() {
    }
    /**
   * Get length of SpringSegment.
   */
    TScalar getLength() const {
        return length;
    }

    /**
   * Get first derivative of leght of SpringSegment w.r.t. time.
   */
    TScalar getDLength() const {
        return dLength;
    }

    /**
   * Calculate length and dlenght/dtime of SpringSegment.
   */
    void calculateGeometry() {
        direction = end2.getCoordinateFrame().r - end1.getCoordinateFrame().r;
        length = direction.norm();
        vRel = end2.getCoordinateFrame().R * end2.getCoordinateFrame().v - end1.getCoordinateFrame().R * end1.getCoordinateFrame().v;
        if (length != 0.0) {
            direction *= (1 / length);
            dLength = vRel.dot(direction);
        } else {
            direction = vRel;
            dLength = direction.norm();
            if (dLength != 0) {
                direction *= (1 / dLength);
            }
        }
    }

    /**
   * Apply force of the spring to the endpoints of the SpringSegment.
   */
    void applyForce(TScalar f) {
        end1.addExternalForceTorqueWCS(direction * f, TVector3::Zero());
        end2.addExternalForceTorqueWCS(-direction * f, TVector3::Zero());
    }

private:
    /// First endpoint of SpringSegment.
    Endpoint & end1;

    /// Second endpoint of SpringSegment.
    Endpoint & end2;

    /// Length of SpringSegment.
    TScalar length;

    /// First derivative of Lenght of SpringSegment.
    TScalar dLength;

    /// Direction of the Segment.
    TVector3 direction;

    TVector3 vRel;
};

Spring3D::Spring3D(const SpringModel & model, Endpoint & end1, Endpoint & end2, const std::string & name)
    : Spring(model, name) {
    points.push_back(&end1);
    points.push_back(&end2);
    segments.push_back(new SpringSegment(end1, end2));
}

Spring3D::Spring3D(const SpringModel & model, std::vector< Endpoint * > points, const std::string & name)
    : Spring(model, name) {
    Endpoint * last = nullptr;
    for (Endpoint * it : points) {
        this->points.push_back(it);
        if (last) {
            segments.push_back(new SpringSegment(*last, *it));
        }
        last = it;
    }
}

Spring3D::Spring3D(const SpringModel & model, const std::string & name)
    : Spring(model, name) {
}

Spring3D & Spring3D::operator<<(Endpoint * e) {
    Endpoint * last = NULL;
    if (points.size() != 0) {
        last = points[points.size() - 1];
    }

    points.push_back(e);

    if (last) {
        segments.push_back(new SpringSegment(*last, *e));
    }

    return *this;
}

void Spring3D::applyForce() {
    TScalar l = 0;
    TScalar dl = 0;
    for (SpringSegment * it : segments) {
        (*it).calculateGeometry();
        l += (*it).getLength();
        dl += (*it).getDLength();
    }
    // added by MH

    Lm = l;
    Vm = dl;

    /*  old Stelzer model
  if (deriveBy == 0)
    // if !deriveMode return Fpee, else if deriveMode return Fde (set valueID to 0)!
*/
    //std::cout << "l " << l << " dl " << dl << std::endl;
    f = model->calculateForce(l, dl); // resulting muscle force
    //std::cout << "f " << f << std::endl;

    ///  Apply force of the spring to the endpoints of the SpringSegment.
    for (SpringSegment * it : segments) {
        (*it).applyForce(f);
    }

    /*    if(deriveMode) {
    switch (deriveBy) {
      case 1:
        f = model->calculateDForceDLength(l, dl);
        break;
      case 2:
        f = model->calculateDForceDVelocity(l, dl);
        break;
      default:
      break;
      // assert(0);
      //  f = 0;  //TODO: what else?
    }*/
}

void Spring3D::resetForce() {
    for (std::vector< Endpoint * >::iterator it = points.begin(); it != points.end(); it++) {
        (**it).setExternalForceTorque(TVector3::Zero(), TVector3::Zero());
    }
}

// added by MH, for old Stelzer
/*
void Spring3D::setDeriveMode(bool dm, unsigned int valueId)
{
  deriveMode = dm;
  deriveBy = valueId;
}

bool Spring3D::getDeriveMode()const{return deriveMode;}
*/

std::vector< Endpoint * > & Spring3D::getPoints() {
    return points;
}

Spring3D::~Spring3D() {
    for (auto i : segments) {
        delete i;
    }
}
} //namespace mbslib
