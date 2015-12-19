% ------------------------------------------------------
% This script runs a parameter identification example with an elastic
% BioRob robot arm. 
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization, and Robotics Group
% Written by Janis Wojtusch, 2015
% ------------------------------------------------------

% Clean up workspace
clc;
clear all;
close all;

% Set parameters
noiseFlag = 1;
stiffnessConstants = [45.0; 40.0; 25.0; 15.0];
dampingConstants = [0.22; 0.18; 0.12; 0.14];
motorPositionAmplitudes = [2.5; 3.0; 3.5; 2.5];
motorPositionFrequences = [2.0; 2.8; 3.2; 1.8];
noisePercentage = [ ...
    0.005, 0.005, 0.005, 0.005; ...
    0.03, 0.03, 0.03, 0.03; ...
    0.1, 0.1, 0.1, 0.1 ...
];
t_end = 20.0;
t_step = 0.01;
x0 = [0; 0; 0; 0; 0; 0; 0; 0];
p0 = [20.0; 20.0; 20.0; 20.0; 0.1; 0.1; 0.1; 0.1];
mu0 = 2.0;
beta0 = 0.3;
beta1 = 0.9;
k_max = 100;
T = 1e-9;

% Load BioRob library
%libraryPath = pwd;
example_library
%libraryName = 'exp_21_biorob_parameter_identification';
loadlibrary(libraryPath, libraryHeaderPath);
%loadlibrary([libraryPath, filesep, 'lib', filesep, libraryName, '.so'], [libraryPath, filesep, 'biorob.h']);

% Create model
calllib(libraryName, 'createModel');

% Compute joint trajectories with reference stiffness and damping constants
fprintf('STATUS: Computing joint trajectories.\n');
interval = 0:t_step:t_end;
options = odeset('MaxStep' , t_step);
[t, x]  = ode15s(@(t, x)applyForwardDynamics(t, x, stiffnessConstants, dampingConstants, motorPositionAmplitudes, motorPositionFrequences, libraryName) , interval , x0 , options);
jointPositions = [x(:, 1), x(:, 3), x(:, 5), x(:, 7)]';
jointVelocities = [x(:, 2), x(:, 4), x(:, 6), x(:, 8)]';
jointAccelerations = zeros(4, length(t));
motorPositions = computeMotorPositions(t', motorPositionAmplitudes, motorPositionFrequences);
for timeIndex = 1:length(t)

    % Set joint states
    positions = jointPositions(:, timeIndex);
    velocities = jointVelocities(:, timeIndex);
    calllib(libraryName, 'setJointStates', ...
        positions, ...
        velocities ...
    );
    
    % Apply forward dynamics
    positions = motorPositions(:, timeIndex);
    calllib(libraryName, 'applyForwardDynamics', ...
        positions, ...
        stiffnessConstants, ...
        dampingConstants ...
    );

    % Get forward dynamics results
    accelerations = zeros(4, 1);
    accelerationsPointer = libpointer('doublePtr', accelerations);
    calllib(libraryName, 'getForwardDynamicsResults', ...
        accelerationsPointer ...
    );
    accelerations = accelerationsPointer.value;
    jointAccelerations(:, timeIndex) = accelerations;

end

% Add noise
if noiseFlag
   
    jointPositionMeasurements = jointPositions + diag(noisePercentage(1, :)) * diag(max(jointPositions, [], 2)) / 3 * randn(size(jointPositions));
    jointVelocityMeasurements = jointVelocities + diag(noisePercentage(2, :)) * diag(max(jointVelocities, [], 2)) / 3 * randn(size(jointVelocities));
    jointAccelerationMeasurements = jointAccelerations + diag(noisePercentage(3, :)) * diag(max(jointAccelerations, [], 2)) / 3 * randn(size(jointAccelerations));
    
else
    
    jointPositionMeasurements = jointPositions;
    jointVelocityMeasurements = jointVelocities;
    jointAccelerationMeasurements = jointAccelerations;
    
end

% Run parameter identification
fprintf('STATUS: Starting parameter identification.\n');
[p, hist] = applyLevenbergMarquardt(@(p)computeResiduals(p, motorPositions, jointPositionMeasurements, jointVelocityMeasurements, jointAccelerationMeasurements, libraryName), p0, mu0, beta0, beta1, k_max, T);

% Unload BioRob library
unloadlibrary(libraryName);
