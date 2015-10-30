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
 * \file mbslib/collision/PointContact.hpp
 * Declaration of mbslib::PointContact
 */
#ifndef __MBSLIB_POINTCONTACT_HPP__
#define __MBSLIB_POINTCONTACT_HPP__

#include <mbslib/elements/endpoint/Endpoint.hpp>

namespace mbslib {

/**
 * \brief a Point contact.
 */
class PointContact {
public:
    /**
   * \brief Constructor.
   *
   * \param endpoint  The endpoint.
   * \param normal    The normal.
   * \param depth     (optional) the depth.
   */
    PointContact(Endpoint & endpoint, const TVector3 & normal, TScalar depth = 0);

    virtual ~PointContact();
    /**
   * \brief Get endpoint.
   *
   * \return  The endpoint.
   */
    Endpoint & getEndpoint();

    /**
   * \brief Get endpoint.
   *
   * \return  The endpoint.
   */
    const Endpoint & getEndpoint() const;

    /**
   * \brief Get penetration depth.
   *
   * \return  The depth.
   */
    TScalar getDepth() const;

    /**
   * \brief Get penetration velocity.
   *
   * \return  The penetration velocity.
   */
    TScalar getPenetrationVelocity() const;

    /**
   * \brief Get normal.
   *
   * \return  The normal local coordinate system.
   */
    const TVector3 & getNormalLocalCoordinateSystem() const;

    /**
   * \brief Get normal.
   *
   * \return  The normal world coordinate system.
   */
    const TVector3 getNormalWorldCoordinateSystem() const;

    /**
   * \brief Get velocity at contact.
   *
   * \return  The velocity local coordinate system.
   */
    const TVector3 getVelocityLocalCoordinateSystem() const;

    /**
   * \brief Get velocity at contact.
   *
   * \return  The velocity world coordinate system.
   */
    const TVector3 getVelocityWorldCoordinateSystem() const;

protected:
    /// The colliding endpoint
    Endpoint * endpoint;

    /// The normal of the contact.
    TVector3 normal;

    /// Depth of the penetration.
    TScalar depth;

}; // class PointContact

} // namespace mbslib

#endif
