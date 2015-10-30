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
 * \file src/example/exp_05_derive_mbs.cpp
 * 
 */
#include <mbslib/elements/compound/MbsCompoundWithBuilder.hpp>
#include <mbslib/utility/DeriveOMat.hpp>

using namespace mbslib;

int main() {
    MbsCompoundWithBuilder mbs;

    mbs.addFixedBase("base");
    Joint1DOF * j1 = mbs.addRevoluteJoint(TVector3(0, 0, 1), 0, 0, 0, 0, "joint1");
    mbs.addFixedTranslation(TVector3(1, 0, 0), "trans1");
    mbs.addEndpoint(1, TMatrix3x3::Zero(), "endpoint1");

    mbs.setGravitation(TVector3(0, -1, 0));

    double indep[] = {1, 1};
    double dep[1];

    DeriveOMat dom(mbs);

    dom.addIndependent("trans1", "rx");
    dom.addIndependent("endpoint1", "mass");
    dom.addDependent("joint1", "tau");

    dom.startTape(indep);
    mbs.doRne();
    dom.endTape(dep);

    std::cout << dep[0] << std::endl;

    const double * value;
    const double * const * J;

    indep[0] = 2;
    value = dom.evaluateFunction(indep);
    J = dom.calculateJacobian(indep);
    std::cout << J[0][0] << " " << J[0][1] << " " << std::endl;
    indep[1] = 2;
    value = dom.evaluateFunction(indep);
    J = dom.calculateJacobian(indep);
    std::cout << J[0][0] << " " << J[0][1] << " " << std::endl;
    indep[0] = 1;
    value = dom.evaluateFunction(indep);
    J = dom.calculateJacobian(indep);
    std::cout << J[0][0] << " " << J[0][1] << " " << std::endl;

    return 0;
}
