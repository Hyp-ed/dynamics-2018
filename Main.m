%% Pod Trajectory Simulation, HypED 2018/19
% This script calculates the trajectory of the pod inside the tube.
% In this script the pod accelerates and decelerates by alternating 
% driving force from the Halbach Wheels.
% In this script, a linear increase/decrease in rpm is assumed, and the 
% rpm cannot exceed the max rpm given by the motor specs.
% NOTE: inertia in power input has been ignored when in efficiency
% calculation.

clear; clc;

%% TODO:
%  - Bisection for breaking distance calculation 

%% Define parameters
% Define basic parameters
dt = 0.05;          % Time step (resolution)
tmax = 60;          % Maximum allowed duration of run
n_wheel = 6;        % Number of wheels
distance_max = 440; % Experimentally found value for maximum distance of accelertaion phase

% Import parameters from './Parameters/HalbachWheel_parameters.xlsx'
halbach_wheel_parameters = importHalbachWheelParameters();


%% Initialize arrays
%  Create all necessary arrays and initialize with 0s for each time step. 
%  This is computationally faster than extending the arrays after each calculation.
time = 0:dt:tmax;                       % Create time array with time step dt and maximum time tmax
v = zeros(1,length(time));              % Velocity of pod
a = zeros(1,length(time));              % Acceleration of pod
distance = zeros(1,length(time));       % Distance travelled
omega = zeros(1,length(time));          % Angular velocity of Halbach wheels
torque = zeros(1,length(time));         % Torque on Halbach wheels
torque_lat = zeros(1,length(time));     % Torque on Halbach wheels from lateral forces
power = zeros(1,length(time));          % Power
power_loss = zeros(1,length(time));     % Power loss
power_input = zeros(1,length(time));    % Power input
efficiency = zeros(1,length(time));     % Power output / Power input
slips = zeros(1,length(time));          % Slip ratio between Halbach wheels and track
f_thrust_wheel = zeros(1,length(time)); % Thrust force from a single Halbach wheel
f_lat_wheel = zeros(1,length(time));    % Lateral force from a single Halbach wheel   
f_x_pod = zeros(1,length(time));        % Net force in direction of track (x) for whole pod
f_y_pod = zeros(1,length(time));        % Net force in lateral direction (y) for whole pod


%% Calculation loop
%  This is the main loop of the script, caluclating the relevant values for
%  each point in time. The function calc_main gets called at each iteration 
%  and handles the phases of the trajectory internally by passing a "phase"
%  variable as the first input argument.
%  phase = 1 -- Acceleration
%  phase = 2 -- Deceleration
%  phase = 3 -- Max RPM

phase = 1; % We start in the acceleration phase

% For each point in time ...
for i = 2:length(time) % Start at i = 2 because values are all init at 1
    %% Phase transitions
    % If we have exceeded the max. RPM we cap the RPM and recalculate
    if (omega(i-1) * 60 / (2*pi)) > halbach_wheel_parameters.m_rpm
        phase = 3; % Max RPM
        
        % Recalculate previous time = i - 1 to avoid briefly surpassing max RPM
        [v,a,distance,omega,torque,torque_lat,power,power_loss,power_input,efficiency,slips,f_thrust_wheel,f_lat_wheel,f_x_pod,f_y_pod] = ...
        calc_main(phase, i - 1, dt, n_wheel, v, a, distance, omega, torque, torque_lat, power, power_loss, power_input, efficiency, slips, ...
                  f_thrust_wheel, f_lat_wheel, f_x_pod, f_y_pod, halbach_wheel_parameters);
    end
    
    % If we have reached the maximum allowed acceleration distance we 
    % transition to deceleration
    if distance(i-1) >= distance_max
        phase = 2; % Deceleration
    end
    
    
    %% Main calculation
    % Calculate for current time = i
    [v,a,distance,omega,torque,torque_lat,power,power_loss,power_input,efficiency,slips,f_thrust_wheel,f_lat_wheel,f_x_pod,f_y_pod] = ...
    calc_main(phase, i, dt, n_wheel, v, a, distance, omega, torque, torque_lat, power, power_loss, power_input, efficiency, slips, ...
              f_thrust_wheel, f_lat_wheel, f_x_pod, f_y_pod, halbach_wheel_parameters);
    
    fprintf("%.2f m, %.2f m/s\n", distance(i), v(i))
    
    %% Exit conditions
    % Stop when speed is 0m/s or time is up
    if v(i) <= 0 || i == length(time)
        % Truncate arrays and create final result structure 
        result = finalizeResults(i, time, distance, v, a, omega * 60 / (2*pi), torque, torque_lat, f_thrust_wheel, f_lat_wheel,...
                                 f_x_pod, f_y_pod, power, power_loss, power_input, efficiency, slips);
        % Break from loop
        break;
    end
end


%% Print some results
% Find max. speed and x force
v_max = max(result.velocity);
v_max_time = find(result.velocity == v_max) * dt - dt;
f_x_max = max(result.pod_x);
torque_max = max(result.torque);
torque_min = min(result.torque);
torque_lat_max = max(result.torque_lat);
% Let's display some stuff for quick viewing
fprintf('\n--------------------RESULTS--------------------\n');
fprintf('\nDuration of run: %.2f s\n', time(i));
fprintf('\nDistance: %.2f m\n', distance(i));
fprintf('\nMaximum speed: %.2f m/s at %.2f s\n', v_max, v_max_time);
fprintf('\nMaximum net x-force: %.2f N\n', f_x_max);
fprintf('\nMaximum thrust torque: %.2f Nm\n', torque_max);
fprintf('\nMinimum thrust torque: %.2f Nm\n', torque_min);
fprintf('\nMaximum lateral torque: %.2f Nm\n', torque_lat_max);

%% Plot the trajectory graphs
plotTrajectory(result.time,result.distance,result.velocity,result.power_input,result.rpm,result.torque);

