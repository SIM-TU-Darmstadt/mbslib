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

#include <mbslib/utility/types.hpp>

#ifndef __MBSLIB_COORDINATE_FRAME_HPP__
#define __MBSLIB_COORDINATE_FRAME_HPP__

namespace mbslib {

/**
 * \brief Description of a coordinate frame.
 */
class CoordinateFrame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Constructor
    CoordinateFrame();

    /// Destructor
    virtual ~CoordinateFrame();

    /// Position of frame.
    TVector3 r;

    /// Orientation of frame
    TMatrix3x3 R;

    /// \brief Spatial representation of velocity
    /// \warning This member is not maintained by all algorithms of mbslib.
    TVector6 spatialVelocity;

    /// \brief Spatial representation of acceleration
    /// \warning This member is not maintained by all algorithms of mbslib.
    TVector6 spatialAcceleration;

    /// Linear velocity of frame
    //Eigen::Block<TVector6,3,1>
    TVector3 v;

    /// Angular velocity of frame
    //Eigen::Block<TVector6,3,1>
    TVector3 omega;

    /// Linear acceleration of frame
    //Eigen::Block<TVector6,3,1>
    TVector3 dotv;

    /// Angular acceleration of frame
    //Eigen::Block<TVector6,3,1>
    TVector3 dotomega;

}; //class CoordianteFrame

} // namespace mbslib

#endif // __MBSLIB_COORDINATE_FRAME_HPP__
