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
% - Move results and plotting into seperate function

%% Import parameters
halbach_wheel_parameters = importHalbachWheelParameters();
halbach_array_parameters = importHalbachArrayParameters();

% Define air gap
g = 0.006;
halbach_wheel_parameters.g = g;

dt = 0.05;
tmax = 80;

angular_acceleration = 111.755; 
rpm_max = 7000;
distance_max = 800;
n_wheel = 4;

m_omega = rpm_max/60*2*pi; % 10000rpm

%% Create time array and set intial conditions
time = 0:dt:tmax;
v = zeros(1,length(time));
a = zeros(1,length(time));
distance = zeros(1,length(time));
omega = zeros(1,length(time));
torque = zeros(1,length(time));
f_drag_wheel = zeros(1,length(time)); % drag force from wheels
f_lat_wheel = zeros(1,length(time)); %lateral force from halbach wheel
f_drag_lev = zeros(1,length(time)); % drag force from skis
f_lift_lev = zeros(1,length(time));
f_drag_pod = zeros(1,length(time)); % force in x direction for the WHOLE POD
f_lift_pod = zeros(1,length(time)); % force in y direction for the WHOLE POD
power = zeros(1,length(time));
efficiency = zeros(1,length(time)); % power output / power input
slips = zeros(1,length(time));
height = zeros(1,length(time));

%% Start for loop for acceleration phase 
for i = 2:length(time)
    halbach_wheel_parameters.vt = v(i-1);
    
    % Find maximum slip and corresponding driving force by minimizing the negative of the driving force.
    [slips(i), f_drag_wheel(i)] = fminbnd(@(x) -fx(x,v(i-1),halbach_wheel_parameters),0,50);
    omega(i) = (slips(i)+v(i-1))/halbach_wheel_parameters.ro; % Default case where omega isn't capped 
    f_drag_wheel(i) = -halbach_wheel_parameters.w*f_drag_wheel(i)*n_wheel; % change sign so that force is positive
    f_lat_wheel(i) = halbach_wheel_parameters.w*fy(slips(i), v(i-1), halbach_wheel_parameters)*n_wheel; 

    % Cap omega, if the angular acceleration is too huge (assume linear increase)
    omega_slope = abs(omega(i)-omega(i-1))/dt; % Calculate slope in change of omega
    if omega_slope > m_omega 
        omega(i) = omega(i-1) + m_omega*dt;
        slips(i) = halbach_wheel_parameters.ro*omega(i) - v(i-1);
        f_drag_wheel(i) = halbach_wheel_parameters.w*n_wheel*fx(slips(i), v(i-1), halbach_wheel_parameters); 
    end
    halbach_wheel_parameters.s = slips(i);
    
    % Calculate total drag force and lift force, which includes lev forces
    [f_lift_lev(i),f_drag_lev(i), height(i)] = FDP_helper_fn_lev_forces(v(i-1), halbach_array_parameters); 
    f_drag_pod(i) = f_drag_wheel(i)-f_drag_lev(i);
    f_lift_pod(i) = f_lift_lev(i);
    
    % Calculate acceleration, velocity and distance
    a(i) = f_drag_pod(i)/halbach_wheel_parameters.M;
    v(i) = v(i-1) + dt*a(i);
    distance(i) = distance(i-1)+dt*v(i); %forward euler (sohuld we use backward euler)
    
    % Torque, power and efficiency
    torque(i) = halbach_wheel_parameters.ro*f_drag_wheel(i)/n_wheel;
    power(i) = f_drag_wheel(i)*v(i); % power output = force * velocity
    power_loss = halbach_wheel_parameters.w*n_wheel*pl(slips(i), v(i), halbach_wheel_parameters);
    power_input = f_drag_wheel(i)*v(i)+power_loss; % ignoring inertia
    efficiency(i) = power(i)/power_input;

    k = i;
    % if reaches max rpm or max distance, go to next loop
    if (omega(i)*9.55 >= rpm_max) || (distance(i) > distance_max)
        break;
    end
end

disp(i)

%% For loop for maintaning rpm at our max rpm
for i = k+1:length(time)
    omega(i) = omega(i-1);
    slips(i) = omega(i)*halbach_wheel_parameters.ro - v(i-1);
    f_drag_wheel(i) = halbach_wheel_parameters.w*n_wheel*fx(slips(i), v(i-1), halbach_wheel_parameters);
    halbach_wheel_parameters.s = slips(i);
    f_lat_wheel(i) = halbach_wheel_parameters.w*fy(slips(i), v(i-1), halbach_wheel_parameters)*n_wheel; 


    % Calculate total drag force and lift force, which includes lev forces
    [f_lift_lev(i),f_drag_lev(i), height(i)] = FDP_helper_fn_lev_forces(v(i-1), halbach_array_parameters); 
    f_drag_pod(i) = f_drag_wheel(i)-f_drag_lev(i);
    f_lift_pod(i) = f_lift_lev(i);
    
    % Acceleration, velocity and distance
    a(i) = f_drag_pod(i)/halbach_wheel_parameters.M;
    v(i) = v(i-1) + dt*a(i);
    distance(i) = distance(i-1)+dt*v(i);
    
    % Torque, power and efficiency
    torque(i) = halbach_wheel_parameters.ro*f_drag_wheel(i)/n_wheel;
    power(i) = f_drag_wheel(i)*v(i); % power output = force * velocity
    power_loss = halbach_wheel_parameters.w*n_wheel*pl(slips(i), v(i), halbach_wheel_parameters);
    power_input = f_drag_wheel(i)*v(i)+power_loss; % ignoring inertia
    efficiency(i) = power(i)/power_input;

    k = i;
    % When max distance is reached, go to next loop 
    if distance(i) > distance_max
        break;
    end
end
    
disp(distance(i));

%% Deceleration phase
for i = k+1:length(time)
    halbach_wheel_parameters.vt = v(i-1);
    
    % Find maximum slip and corresponding driving force by minimizing the positive of the driving force.
    [slips(i),f_drag_wheel(i)] = fminbnd(@(x) fx(x,v(i-1),halbach_wheel_parameters),-50,0);
    
    % Default case where omega isn't capped 
    f_drag_wheel(i) = halbach_wheel_parameters.w*f_drag_wheel(i)*n_wheel; % change sign so that force is positive
    omega(i) = (slips(i)+v(i-1))/halbach_wheel_parameters.ro;
    f_lat_wheel(i) = halbach_wheel_parameters.w*fy(slips(i), v(i-1), halbach_wheel_parameters)*n_wheel; 

    
    % Cap omega, if the angular deceleration is too huge (assume linear increase)
    omega_slope = abs(omega(i)-omega(i-1))/dt; % Calculate slope in change of omega
    if omega_slope > angular_acceleration 
        omega(i) = omega(i-1) - angular_acceleration*dt;
        slips(i) = halbach_wheel_parameters.ro*omega(i) - v(i-1);
        f_drag_wheel(i) = halbach_wheel_parameters.w*n_wheel*fx(slips(i), v(i-1), halbach_wheel_parameters); 
    end
    
    halbach_wheel_parameters.s = slips(i);
    
    % Calculate total drag force and lift force, which includes lev forces
    [f_lift_lev(i),f_drag_lev(i), height(i)] = FDP_helper_fn_lev_forces(v(i-1), halbach_array_parameters); 
    f_drag_pod(i) = f_drag_wheel(i)-f_drag_lev(i);
    f_lift_pod(i) = f_lift_lev(i);

    % Acceleration, velocity and distance
    a(i) = f_drag_pod(i)/halbach_wheel_parameters.M;
    v(i) = v(i-1) + dt*a(i);
    distance(i) = distance(i-1)+dt*v(i);
   
    % Torque, power and efficiency
    torque(i) = halbach_wheel_parameters.ro*f_drag_wheel(i)/n_wheel;
    power(i) = f_drag_wheel(i)*v(i); % power output = force * velocity
    power_loss = halbach_wheel_parameters.w*n_wheel*pl(slips(i), v(i), halbach_wheel_parameters);
    power_input = f_drag_wheel(i)*v(i)+power_loss; % ignoring inertia
    efficiency(i) = power(i)/power_input;

    k = i;
    % Stop when speed is 0. 
    if v(i) <= 0
        break;
    end
end

%% Truncate arrays and create struct array to store results 
result = finalizeResults(k,time,distance,v,a,9.55*omega,torque,f_drag_wheel,f_drag_lev,f_lift_lev,...
                         f_drag_pod,f_lift_pod,power,efficiency,slips,f_lat_wheel,height);

%% Plot the trajectory graphs
plotTrajectory(result.time,result.distance,result.velocity);