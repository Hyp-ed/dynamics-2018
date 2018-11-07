%% Simulate Pod Trajectory, HypED 2018/19
% This script calculates the trajectory of the pod inside the tube.
% In this script the pod accelerates and decelerates by alternating 
% driving force from the Halbach Wheels.
% In this script, a linear increase/decrease in rpm is assumed, and the 
% rpm cannot exceed the max rpm given by the motor specs.
% NOTE: inertia in power input has been ignored when in efficiency
% calculation.

clear; clc;


%% TODO: 
% - Fix halbach array parameters (rewrite excel functions in matlab)
% - Bisection for breaking distance calculation 
% - Comment everything!
% - Move three phases into single function that gets executed conditionally
% - Remove levitation ski forces

%% Define parameters
% Define basic parameters
dt = 0.05;          % Time step (resolution)
tmax = 17;          % Maximum allowed duration of run
n_wheel = 6;        % Number of wheels
distance_max = 900; % Experimentally found value for maximum distance of accelertaion phase

% Import parameters from .xlsx
halbach_wheel_parameters = importHalbachWheelParameters(); % './Parameters/HalbachWheel_parameters.xlsx'
halbach_array_parameters = importHalbachArrayParameters(); % './Parameters/HalbachArray_parameters.xlsx'

% Define max. possible angular velocity of Halbach wheel for imported max. rpm
m_omega = halbach_wheel_parameters.m_rpm/60*2*pi;


%% Initialize arrays
%  Create all necessary arrays and initialize with 0s for each time step. 
%  This is computationally faster than extending the arrays after each calculation.
time = 0:dt:tmax;                       % Create time array with time step dt and maximum time tmax
v = zeros(1,length(time));              % Velocity of pod
a = zeros(1,length(time));              % Acceleration of pod
distance = zeros(1,length(time));       % Distance travelled
omega = zeros(1,length(time));          % Angular velocity of Halbach wheels
torque = zeros(1,length(time));         % Torque on Halbach wheels
power = zeros(1,length(time));          % Power
efficiency = zeros(1,length(time));     % Power output / Power input
slips = zeros(1,length(time));          % Slip between Halbach wheels and track
f_thrust_wheel = zeros(1,length(time)); % Thrust force from Halbach wheels
f_lat_wheel = zeros(1,length(time));    % Lateral force from Halbach wheels   
f_x_pod = zeros(1,length(time));        % Force in direction of track (x) for whole pod
f_y_pod = zeros(1,length(time));        % Force in vertical direction (y) for whole pod (UNUSED)

%% Start for loop for acceleration phase 
for i = 2:length(time)
    halbach_wheel_parameters.vt = v(i-1);
    
    % Find maximum slip and corresponding driving force by minimizing the negative of the driving force.
    [slips(i), f_thrust_wheel(i)] = fminbnd(@(x) -fx(x,v(i-1),halbach_wheel_parameters),0,50);
    
    omega(i) = (slips(i)+v(i-1))/halbach_wheel_parameters.ro; % Default case where omega isn't capped 
    f_thrust_wheel(i) = -halbach_wheel_parameters.w*f_thrust_wheel(i)*n_wheel; % change sign so that force is positive
    f_lat_wheel(i) = halbach_wheel_parameters.w*fy(slips(i), v(i-1), halbach_wheel_parameters)*n_wheel; 
    
    % Cap omega, if the angular acceleration is too huge (assume linear increase)
    omega_slope = abs(omega(i)-omega(i-1))/dt; % Calculate slope in change of omega
    if omega_slope > halbach_wheel_parameters.m_alpha
        omega(i) = omega(i-1) + halbach_wheel_parameters.m_alpha*dt;
        slips(i) = halbach_wheel_parameters.ro*omega(i) - v(i-1);
        f_thrust_wheel(i) = halbach_wheel_parameters.w*n_wheel*fx(slips(i), v(i-1), halbach_wheel_parameters); 
    end
    halbach_wheel_parameters.s = slips(i);
    
    % Calculate total drag force
    f_x_pod(i) = f_thrust_wheel(i);
    
    % Calculate acceleration, velocity and distance
    a(i) = f_x_pod(i)/halbach_wheel_parameters.M;
    v(i) = v(i-1) + dt*a(i);
    distance(i) = distance(i-1)+dt*v(i); %forward euler (should we use backward euler)
    
    % Torque, power and efficiency
    torque(i) = halbach_wheel_parameters.ro*f_thrust_wheel(i)/n_wheel;
    power(i) = f_thrust_wheel(i)*v(i); % power output = force * velocity
    power_loss = halbach_wheel_parameters.w*n_wheel*pl(slips(i), v(i), halbach_wheel_parameters);
    power_input = f_thrust_wheel(i)*v(i)+power_loss; % ignoring inertia
    efficiency(i) = power(i)/power_input;

    k = i;
    % if reaches max rpm or max distance, go to next loop
    if (omega(i)*9.55 >= m_rpm) || (distance(i) > distance_max)
        break;
    end
end

%disp(i)

%% For loop for maintaning rpm at our max rpm
for i = k+1:length(time)
    omega(i) = omega(i-1);
    slips(i) = omega(i)*halbach_wheel_parameters.ro - v(i-1);
    f_thrust_wheel(i) = halbach_wheel_parameters.w*n_wheel*fx(slips(i), v(i-1), halbach_wheel_parameters);
    halbach_wheel_parameters.s = slips(i);
    f_lat_wheel(i) = halbach_wheel_parameters.w*fy(slips(i), v(i-1), halbach_wheel_parameters)*n_wheel; 


    % Calculate total drag force
    f_x_pod(i) = f_thrust_wheel(i);
    
    % Acceleration, velocity and distance
    a(i) = f_x_pod(i)/halbach_wheel_parameters.M;
    v(i) = v(i-1) + dt*a(i);
    distance(i) = distance(i-1)+dt*v(i);
    
    % Torque, power and efficiency
    torque(i) = halbach_wheel_parameters.ro*f_thrust_wheel(i)/n_wheel;
    power(i) = f_thrust_wheel(i)*v(i); % power output = force * velocity
    power_loss = halbach_wheel_parameters.w*n_wheel*pl(slips(i), v(i), halbach_wheel_parameters);
    power_input = f_thrust_wheel(i)*v(i)+power_loss; % ignoring inertia
    efficiency(i) = power(i)/power_input;

    k = i;
    % When max distance is reached, go to next loop 
    if distance(i) > distance_max
        break;
    end
end
    
%disp(distance(i));

%% Deceleration phase
for i = k+1:length(time)
    halbach_wheel_parameters.vt = v(i-1);
    
    % Find maximum slip and corresponding driving force by minimizing the positive of the driving force.
    [slips(i),f_thrust_wheel(i)] = fminbnd(@(x) fx(x,v(i-1),halbach_wheel_parameters),-50,0);
    
    % Default case where omega isn't capped 
    f_thrust_wheel(i) = halbach_wheel_parameters.w*f_thrust_wheel(i)*n_wheel;
    omega(i) = (slips(i)+v(i-1))/halbach_wheel_parameters.ro;
    f_lat_wheel(i) = halbach_wheel_parameters.w*fy(slips(i), v(i-1), halbach_wheel_parameters)*n_wheel; 

    
    % Cap omega, if the angular deceleration is too huge (assume linear increase)
    omega_slope = abs(omega(i)-omega(i-1))/dt; % Calculate slope in change of omega
    if omega_slope > halbach_wheel_parameters.m_alpha 
        omega(i) = omega(i-1) - halbach_wheel_parameters.m_alpha*dt;
        slips(i) = halbach_wheel_parameters.ro*omega(i) - v(i-1);
        f_thrust_wheel(i) = halbach_wheel_parameters.w*n_wheel*fx(slips(i), v(i-1), halbach_wheel_parameters); 
    end
    
    halbach_wheel_parameters.s = slips(i);
    
    % Calculate total drag force
    f_x_pod(i) = f_thrust_wheel(i);

    % Acceleration, velocity and distance
    a(i) = f_x_pod(i)/halbach_wheel_parameters.M;
    v(i) = v(i-1) + dt*a(i);
    distance(i) = distance(i-1)+dt*v(i);
   
    % Torque, power and efficiency
    torque(i) = halbach_wheel_parameters.ro*f_thrust_wheel(i)/n_wheel;
    power(i) = f_thrust_wheel(i)*v(i); % power output = force * velocity
    power_loss = halbach_wheel_parameters.w*n_wheel*pl(slips(i), v(i), halbach_wheel_parameters);
    power_input = f_thrust_wheel(i)*v(i)+power_loss; % ignoring inertia
    efficiency(i) = power(i)/power_input;

    k = i;
    % Stop when speed is 0. 
    if v(i) <= 0
        break;
    end
end

%% Truncate arrays and create struct array to store results 
result = finalizeResults(k, time, distance, v, a, omega * 60 / (2*pi), torque, f_thrust_wheel, f_lat_wheel,...
                         f_x_pod, f_y_pod, power, efficiency, slips);

%% Plot the trajectory graphs
plotTrajectory(result.time,result.distance,result.velocity);