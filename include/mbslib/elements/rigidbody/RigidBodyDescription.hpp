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
 * \file mbslib/elements/rigidbody/RigidBodyDescription.hpp
 * Declaration of mbslib::RigidBodyDescription
 */
#ifndef __MBSLIB_RIGIDBODYDESCRIPTION_HPP__
#define __MBSLIB_RIGIDBODYDESCRIPTION_HPP__

#include <mbslib/elements/object/MbsObjectOneSucc.hpp>

namespace mbslib {

/**
 * \brief Description of the rigid body. \todo kommentieren
 */
class RigidBodyDescription {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
   * \brief Default constructor.
   */
    RigidBodyDescription();

    /**
   * \brief Constructor.
   *
   * \param r   The position.
   * \param com The center of mass.
   * \param m   The mass.
   * \param I   The intertia matrix.
   */
    RigidBodyDescription(const TVector3 & r, const TVector3 & com, TScalar m, const TMatrix3x3 & I);

    virtual ~RigidBodyDescription();
    /**
   * \brief Mirror the description at a plane with a given normal which intersects (0 0 0).
   *  
   *  This method is useful when creating symmetric mbs. It will change the center of mass
   *  accordingly.
   *
   * \param normal  Normal vector of plane to mirror this RigidBodyDescription on.
   *
   * \return  the rigid body description.
   */
    RigidBodyDescription mirror(const TVector3 & normal);

    /// Translation
    TVector3 r;

    /// Center of mass.
    TVector3 com;

    /// Mass;
    TScalar m;

    /// Inertia at CoM.
    TMatrix3x3 I;
}; //class RigidBodyDescription

} //namespace mbslib

#endif
