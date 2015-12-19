% ------------------------------------------------------
% This script applies the forward dynamics of the elastic BioRob robot arm
% and returns the state vector.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization, and Robotics Group
% Written by Janis Wojtusch, 2015
% ------------------------------------------------------

function dxdt = applyForwardDynamics(t, x, stiffnessConstants, dampingConstants, motorPositionAmplitudes, motorPositionFrequences, libraryName)

% Set joint states
jointPositions = [x(1); x(3); x(5); x(7)];
jointVelocities = [x(2); x(4); x(6); x(8)];
calllib(libraryName, 'setJointStates', ...
    jointPositions, ...
    jointVelocities ...
);

% Compute motor positions
motorPositions = computeMotorPositions(t, motorPositionAmplitudes, motorPositionFrequences);

% Apply forward dynamics
calllib(libraryName, 'applyForwardDynamics', ...
    motorPositions, ...
    stiffnessConstants, ...
    dampingConstants ...
);

% Get forward dynamics results
jointAccelerations = zeros(4, 1);
jointAccelerationsPointer = libpointer('doublePtr', jointAccelerations);
calllib(libraryName, 'getForwardDynamicsResults', ...
    jointAccelerationsPointer ...
);
jointAccelerations = jointAccelerationsPointer.value;

dxdt = [...
    jointVelocities(1); jointAccelerations(1); ...
    jointVelocities(2); jointAccelerations(2); ...
    jointVelocities(3); jointAccelerations(3); ...
    jointVelocities(4); jointAccelerations(4) ...
];

end

