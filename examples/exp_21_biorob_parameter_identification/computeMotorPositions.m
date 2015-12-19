% ------------------------------------------------------
% This function computes the motor positions for given amplitudes,
% frequences and times.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization, and Robotics Group
% Written by Janis Wojtusch, 2015
% ------------------------------------------------------

function motorPositions = computeMotorPositions(t, motorPositionAmplitudes, motorPositionFrequences)

% Compute motor positions
motorPositions = [ ...
    motorPositionAmplitudes(1) * sin(motorPositionFrequences(1) * t); ...
    motorPositionAmplitudes(2) * cos(motorPositionFrequences(2) * t); ...
    -motorPositionAmplitudes(3) * sin(motorPositionFrequences(3) * t); ...
    -motorPositionAmplitudes(4) * cos(motorPositionFrequences(4) * t) ...
];

end

