%% Pod Trajectory Simulation, HypED 2018/19
% This script calculates the trajectory of the pod inside the tube by
% calculating the force from the Halbach wheel propulsion module. A linear 
% increase/decrease in RPM is assumed (capped by a given maximum angular
% acceleration), and the rpm cannot exceed the max RPM given by the motor 
% specifications. Inertia in power input has been ignored when in efficiency
% calculation.
%
% NOTE ABOUT TIME STEP (dt):
% For quick estimates a time step of ~0.1s is sufficient. 
% For more accurate results use a time step of 0.05s or smaller.

clear; clc;

%% Parameters
% Script mode
useMaxAccDistance = false;          
maxAccDistance = 1100;

% Import parameters from './Parameters/HalbachWheel_parameters.xlsx'
halbach_wheel_parameters = importHalbachWheelParameters();

% Define basic parameters
dt = 1/20;                          % Time step (see note above)
tmax = 60;                          % Maximum allowed duration of run
n_wheel = 6;                        % Number of wheels
deceleration_total = 2.2 * 9.81;    % Braking deceleration from EmBrakes
stripe_dist = 100 / 3.281;          % Distance between stripes
number_of_stripes = floor(halbach_wheel_parameters.l / stripe_dist); % Total number of stripes we will detect

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
stripes = zeros(1,number_of_stripes);   % Indices at which we detect each stripe

%% Calculation loop
%  This is the main loop of the script, caluclating the relevant values for
%  each point in time. The function calc_main gets called at each iteration 
%  and handles the phases of the trajectory internally by passing a "phase"
%  variable as the first input argument.
%  phase = 1 -- Acceleration
%  phase = 2 -- Deceleration
%  phase = 3 -- Max RPM

phase = 1;          % We start in the acceleration phase
stripe_count = 0;   % Initially we have counted 0 stripes

% For each point in time ...
for i = 2:length(time) % Start at i = 2 because values are all init at 1
    %% Phase transitions
    % If we have exceeded the max. RPM we cap the RPM and recalculate
    if (omega(i-1) * 60 / (2 * pi)) > halbach_wheel_parameters.m_rpm
        phase = 3; % Max RPM
        
        % Recalculate previous time = i - 1 to avoid briefly surpassing max RPM
        [v,a,distance,omega,torque,torque_lat,power,power_loss,power_input,efficiency,slips,f_thrust_wheel,f_lat_wheel,f_x_pod,f_y_pod] = ...
        calc_main(phase, i - 1, dt, n_wheel, v, a, distance, omega, torque, torque_lat, power, power_loss, power_input, efficiency, slips, ...
                  f_thrust_wheel, f_lat_wheel, f_x_pod, f_y_pod, halbach_wheel_parameters, deceleration_total);
    end
    
    % If we have reached the maximum allowed acceleration distance we 
    % transition to deceleration
    if (useMaxAccDistance)
        if distance(i-1) >= (maxAccDistance)
            phase = 2; % Deceleration
        end
    else
        braking_dist = (max(v))^2/(2*deceleration_total);
        if distance(i-1) >= (halbach_wheel_parameters.l - braking_dist)
            phase = 2; % Deceleration
        end
    end
    
    %% Main calculation
    % Calculate for current time = i
    [v,a,distance,omega,torque,torque_lat,power,power_loss,power_input,efficiency,slips,f_thrust_wheel,f_lat_wheel,f_x_pod,f_y_pod] = ...
    calc_main(phase, i, dt, n_wheel, v, a, distance, omega, torque, torque_lat, power, power_loss, power_input, efficiency, slips, ...
              f_thrust_wheel, f_lat_wheel, f_x_pod, f_y_pod, halbach_wheel_parameters, deceleration_total);
    
    fprintf("%.2f ft, %.2f m, %.2f m/s, %.2f s\n", distance(i) * 3.281, distance(i), v(i), time(i))
    
    %% Stripes
    if (distance(i) >= (1 + stripe_count) * stripe_dist)
        stripes(1 + stripe_count) = i;
        stripe_count = stripe_count + 1;
    end
    
    %% Exit conditions
    % Stop when speed is 0m/s or time is up
    if v(i) <= 0 || i == length(time)
        % Truncate arrays and create final result structure 
        result = finalizeResults(i, time, distance, v, a, omega * 60 / (2 * pi), torque, torque_lat, f_thrust_wheel, f_lat_wheel,...
                                 f_x_pod, f_y_pod, power, power_loss, power_input, efficiency, slips);
        % Break from loop
        break;
    end
end


%% Print some results
% Find max. speed and x force
v_max = max(result.velocity);
v_max_time = find(result.velocity == v_max) * dt - dt;
rpm_max = max(result.rpm);
f_x_max = max(result.pod_x);
f_x_min = min(result.pod_x);
torque_max = max(result.torque);
torque_min = min(result.torque);
torque_lat_max = max(result.torque_lat);
% Let's display some stuff for quick viewing
fprintf('\n--------------------RESULTS--------------------\n');
fprintf('\nDuration of run: %.2f s\n', time(i));
fprintf('\nDistance: %.2f m\n', distance(i));
fprintf('\nMaximum speed: %.2f m/s at %.2f s\n', v_max, v_max_time);
fprintf('\nMaximum RPM: %i\n', rpm_max);
fprintf('\nMaximum net thrust force per wheel: %.2f N\n', f_x_max/n_wheel);
fprintf('\nMaximum net lateral force per wheel: %.2f N\n', max(f_y_pod)/n_wheel);
fprintf('\nMaximum thrust torque: %.2f Nm\n', torque_max);
fprintf('\nMaximum braking torque: %.2f Nm\n', torque_min);
fprintf('\nMaximum lateral torque: %.2f Nm\n', torque_lat_max);
fprintf('\nPower per motor: %.2f W\n', max(power_input)/n_wheel);

%% Plot the trajectory graphs
plotTrajectory(result);

% Plot separate Thrust vs RPM plot
figure(8);
plot(result.rpm, result.pod_x); axis tight; ylim([0 2000]); title('Thrust vs RPM'); ylabel('Thrust(N)'); xlabel('RPM');
hold on;