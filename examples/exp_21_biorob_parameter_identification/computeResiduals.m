% ------------------------------------------------------
% This script computes the residuals and jacobians for given a given
% parameter set.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization, and Robotics Group
% Written by Janis Wojtusch, 2015
% ------------------------------------------------------

function [r, J] = computeResiduals(p, motorPositions, jointPositions, jointVelocities, jointAccelerations, libraryName)

o = size(jointAccelerations, 1);
m = size(jointAccelerations, 2);
n = length(p);
jointAccelerationEstimates = zeros(o, m);
jacobianMatrix = zeros([n, m * o]);
for timeIndex = 1:m

    % Set joint states
    positions = jointPositions(:, timeIndex);
    velocities = jointVelocities(:, timeIndex);
    calllib(libraryName, 'setJointStates', ...
        positions, ...
        velocities ...
    );

    % Apply forward dynamics
    positions = motorPositions(:, timeIndex);
    stiffnessConstants = p(1:4);
    dampingConstants = p(5:8);
    calllib(libraryName, 'applyForwardDynamics', ...
        positions, ...
        stiffnessConstants, ...
        dampingConstants ...
    );

    % Get forward dynamics results
    accelerations = zeros(o, 1);
    accelerationsPointer = libpointer('doublePtr', accelerations);
    calllib(libraryName, 'getForwardDynamicsResults', ...
        accelerationsPointer ...
    );
    accelerations = accelerationsPointer.value;
    jointAccelerationEstimates(:, timeIndex) = accelerations;
    
    % Get Jacobians
    jacobian = zeros(n * o, 1);
    jacobianPointer = libpointer('doublePtr', jacobian);
    calllib(libraryName, 'getForwardDynamicsJacobian', ...
        jacobianPointer ...
    );
    jacobian = jacobianPointer.value;
    indices = 1:o:(o * n);
    for dataIndex = 1:o
    
        jacobianMatrix(:, ((dataIndex - 1) * m + timeIndex)) = jacobian(indices + dataIndex - 1);
    
    end
    
end
r = reshape((jointAccelerationEstimates - jointAccelerations)', m * o, 1);
J = jacobianMatrix;

end

