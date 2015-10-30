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
 * \file mbslib/elements/joint/JointNDOF.hpp
 * Declaration of mbslib::JointNDOF
 */
#ifndef __MBSLIB_JOINTNDOF_HPP__
#define __MBSLIB_JOINTNDOF_HPP__

#include <mbslib/elements/joint/Joint.hpp>

#include <mbslib/utility/mathtools.hpp>

namespace mbslib {

/**
 * \brief an n dimensional joint.
 */
template < int N >
class JointNDOF : public Joint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
   * \brief Defines an alias representing an n-vector.
   */
    typedef Eigen::Matrix< TScalar, N, 1 > TVectorN;

    /**
   * \brief Get DOF of element.
   *
   * \return  The degrees of freedom.
   */
    virtual int getDOF() const {
        return N;
    }

#if 0

  /**
   * \brief First (outward) sweep of ABA.
   */
  virtual void doABASweep1fwd(){                                                                                                                                                                                           \
    /* vJ */ TVector6 vJ = S * abaGetDotQ();                                                                                                                                                                            \
    /* cJ */ TVector6 cJ = abaGetCircS() * abaGetDotQ();                                                                                                                                                                \
    /* v  */ cof.spatialVelocity = makeVelocityTransform( getRelativeOrientation().transpose(), -getRelativeOrientation().transpose() * getRelativePosition() ) * pred->getCoordinateFrame().spatialVelocity + vJ;      \
    /* c  */ c = cJ + spatialCross(cof.spatialVelocity, vJ);                                                                                                                                                            \
  }

  /**
   * \brief Second (inward) sweep of ABA.
   */
  virtual void doABASweep2fwd();

  /**
   * \brief Third (outward) sweep of ABA.
   */
  virtual void doABASweep3fwd();
#endif

    virtual ~JointNDOF() {
    }

protected:
    /**
   * \brief Constructor.
   *
   * \param pred                The predecessor.
   * \param jointId             Identifier for the joint.
   * \param stateVectorPosition The state vector position.
   * \param name                (optional) the name.
   */
    JointNDOF(MbsObject & pred, int jointId, int stateVectorPosition, const std::string & name = "")
        : Joint(pred, jointId, stateVectorPosition, name)
        , S(Eigen::Matrix< TScalar, 6, N >::Zero())
        , U(Eigen::Matrix< TScalar, 6, N >::Zero())
        , D(Eigen::Matrix< TScalar, N, N >::Zero()) {
        c.setZero();
        u.setZero();
    }

    /**
   * \brief Gets aba tau.
   *
   * \return  . \todo kommentieren
   */
    virtual TVectorN abaGetTau() = 0;

    /**
   * \brief \todo kommentieren.
   *
   * \return  .
   */
    virtual TVectorN abaGetDotQ() = 0;

    /**
   * \brief \todo kommentieren
   *
   * \return  .
   */
    virtual TVectorN abaGetDDotQ() = 0;

    /**
   * \brief todo kommentieren.
   *
   * \param tau \todo kommentieren.
   */
    virtual void abaSetTau(const TVectorN & tau) = 0;

    /**
   * \brief todo kommentieren
   *
   * \param tau \todo kommentieren.
   */
    virtual void abaSetDotQ(const TVectorN & tau) = 0;

    /**
   * \brief \todo kommentieren
   *
   * \param tau \todo kommentieren.
   */
    virtual void abaSetDDotQ(const TVectorN & tau) = 0;

    /**
   * \brief \todo kommentieren
   *
   * \return  .\todo kommentieren.
   */
    virtual Eigen::Matrix< TScalar, 6, N > abaGetCircS() {
        return Eigen::Matrix< TScalar, 6, N >::Zero();
    }

    /// joint axis representation for ABA
    Eigen::Matrix< TScalar, 6, N > S;

    /// temporary value for ABA, calcualted in sweep one, used in sweeps two and three
    TVector6 c;

    /// temporary value for ABA, calcualted in sweep one, used in sweeps two and three
    Eigen::Matrix< TScalar, 6, N > U;

    /// temporary value for ABA, calcualted in sweep one, used in sweeps two and three
    Eigen::Matrix< TScalar, N, N > D;

    /// temporary value for ABA, calcualted in sweep one, used in sweeps two and three
    TVectorN u;

}; // class JointNDOF

} // namespace mbslib

#endif
