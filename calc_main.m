function [v,a,distance,omega,torque,power,efficiency,slips,f_thrust_wheel,f_lat_wheel,f_x_pod,f_y_pod] = calc_main(phase,i,dt,n_wheel,m_omega,m_alpha,v,a,distance,omega,torque,power,efficiency,slips,f_thrust_wheel,f_lat_wheel,f_x_pod,f_y_pod,halbach_array_parameters,halbach_wheel_parameters)
% CALC_MAIN Calculates trajectory values at each point in time.
% calc_main gets called at each iteration and handles the phases of the 
% trajectory via a passed phase input argument (first).
% phase = 1 -- Acceleration
% phase = 2 -- Deceleration
% phase = 3 -- Max RPM

% Calculate Halbach wheel thrust force and slip
switch phase
    case 1 % Acceleration
        % Find maximum slip and corresponding driving force by minimizing the negative of the driving force.
        [slips(i), f_thrust_wheel(i)] = fminbnd(@(x) -fx(x,v(i-1),halbach_wheel_parameters),0,50);
        f_thrust_wheel(i) = -f_thrust_wheel(i); % Change sign so that force is positive
        
        % Calculate angular velocity of Halbach wheels
        omega(i) = (slips(i)+v(i-1))/halbach_wheel_parameters.ro; 
    case 2 % Deceleration
        % Find minimum (max. negative) slip and corresponding braking force by minimizing the driving force.
        [slips(i), f_thrust_wheel(i)] = fminbnd(@(x) fx(x,v(i-1),halbach_wheel_parameters),-50,0);    
        
        % Calculate angular velocity of Halbach wheels
        omega(i) = (slips(i)+v(i-1))/halbach_wheel_parameters.ro;
    case 3 % Max RPM
        % Set omega to max. omega
        omega(i) = m_omega;
        slips(i) = omega(i)*halbach_wheel_parameters.ro - v(i-1);
        f_thrust_wheel(i) = fx(slips(i), v(i-1), halbach_wheel_parameters);

end

% Cap angular velocity if the angular acceleration is too big (assume linear increase)
alpha = abs(omega(i)-omega(i-1))/dt; % Calculate angular acceleration in change of omega
if alpha > m_alpha
    % Recalculate omega for max. angular acceleration
    switch phase
        case 1 % Acceleration    
            omega(i) = omega(i-1) + m_alpha*dt;
        case 2 % Deceleration
            omega(i) = omega(i-1) - m_alpha*dt;
    end
    % Recalculate slip and Halbach wheel thrust
    slips(i) = halbach_wheel_parameters.ro*omega(i) - v(i-1);
    f_thrust_wheel(i) = fx(slips(i), v(i-1), halbach_wheel_parameters);
end

% Calculate total x forces
f_x_pod(i) = f_thrust_wheel(i)*halbach_wheel_parameters.w*n_wheel;

% Calculate lateral force from Halbach wheel and total y force
f_lat_wheel(i) = fy(slips(i), v(i-1), halbach_wheel_parameters);
f_y_pod(i) = f_lat_wheel(i)*halbach_wheel_parameters.w*n_wheel;

% Calculate acceleration, velocity and distance
a(i) = f_x_pod(i)/halbach_wheel_parameters.M;
v(i) = v(i-1) + dt*a(i);
distance(i) = distance(i-1)+dt*v(i);

% Calculate torque, power and efficiency
torque(i) = halbach_wheel_parameters.ro*f_thrust_wheel(i)/n_wheel;
power(i) = f_thrust_wheel(i)*v(i); % power output = force * velocity
power_loss = halbach_wheel_parameters.w*n_wheel*pl(slips(i), v(i), halbach_wheel_parameters);
power_input = f_thrust_wheel(i)*v(i)+power_loss; % ignoring inertia
efficiency(i) = power(i)/power_input;

end

