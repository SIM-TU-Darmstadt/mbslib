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
 * \file mbslib/collision/CollisionDetector.hpp
 * Declaration of mbslib ::CollisionDetector
 */
#ifndef __MBSLIB_COLLISIONDETECTOR_HPP__
#define __MBSLIB_COLLISIONDETECTOR_HPP__

#include <mbslib/collision/PointContact.hpp>
#include <mbslib/elements/force/ForceGenerator.hpp>
#include <mbslib/collision/PointContactHandler.hpp>

#include <vector>

namespace mbslib {

/**
 * \brief Collision detector.
 */
class CollisionDetector : public ForceGenerator {
public:
    /**
   * \brief Default constructor.
   */
    CollisionDetector();

    /**
   * \brief Constructor.
   *  
   *  Sets collision plane defined by point p and normal n.
   *
   * \param p0  The plane offset.
   * \param n0  The plane normal.
   */
    CollisionDetector(const TVector3 & p0, const TVector3 & n0);

    virtual ~CollisionDetector();
    /**
   * \brief Add an endpoint for contact-detection.
   *
   * \param e The endpoint.
   */
    void addEndpoint(Endpoint & e);

    /**
   * \brief Add an endpoint for contact-detection and an contact-handler.
   *
   * \param  e   The endpoint.
   * \param  ch  The contact handler.
   */
    void addEndpoint(Endpoint & e, PointContactHandler & ch);

    /**
   * \brief Detect contacts of current endpoints.
   */
    void detectContacts();

    /**
   * \brief Get list of currently detected contacts.
   *
   * \return  The contacts.
   */
    const std::vector< PointContact > & getContacts() const;

    /**
   * \brief Create forces by PointContactHandlers.
   */
    void applyForce();

    /**
   * \brief Reset forces by PointContactHandlers.
   */
    void resetForce();

    /**
   * \brief Get name of this.
   *
   * \return  The name.
   */
    virtual const std::string & getName() const;

    /**
   * \brief Gets the integrator.
   *
   * \return  null, always
   */
    virtual IIntegrate * getIntegrator();

protected:
    /// Point of the collision plane.
    TVector3 p0;

    /// Normal vector of the collision plane.
    TVector3 n0;

    /// Endpoints to detect contacts for.
    std::vector< std::pair< Endpoint *, PointContactHandler * > > endpoints;

    /// Detected contacts.
    std::vector< PointContact > contacts;

    /// Pairs of handler and contacts
    std::vector< std::pair< PointContact, PointContactHandler * > > contactHandlers;

    /// Applied forces
    std::vector< std::pair< PointContact, PointContactHandler * > > appliedForces;

    /// Name of this.
    std::string name;

}; //class CollisionDetector

} //namespace mbslib

#endif
