function [v,a,distance,omega,torque,torque_lat,power,power_loss,power_input,efficiency,slips,f_thrust_wheel,f_lat_wheel,f_x_pod,f_y_pod] = calc_main(phase,i,dt,n_wheel,v,a,distance,omega,torque,torque_lat,power,power_loss,power_input,efficiency,slips,f_thrust_wheel,f_lat_wheel,f_x_pod,f_y_pod,halbach_wheel_parameters, a_embrakes)
% CALC_MAIN Calculates trajectory values at each point in time.
% calc_main gets called at each iteration and handles the phases of the 
% trajectory via a passed phase input argument (first).
% phase = 1 -- Acceleration
% phase = 2 -- Deceleration
% phase = 3 -- Max RPM
% @author      Andreas Malekos, Rafael Anderka

    % Calculate slip, Halbach wheel thrust force, torque and angular velocity
    switch phase
        case 1 % Acceleration
            % Find maximum slip and corresponding driving force by minimizing the negative of the driving force
            [slips(i), f_thrust_wheel(i)] = fminbnd(@(x) -fx(x,v(i-1),halbach_wheel_parameters),0,50);
            f_thrust_wheel(i) = -f_thrust_wheel(i); % Change sign so that force is positive
            
            % Calculate angular velocity of Halbach wheels
            omega(i) = (slips(i)+v(i-1))/halbach_wheel_parameters.ro; 
        case 2 % Deceleration using EmBrakes
            omega(i) = 0;
            f_thrust_wheel(i) = 0;
            slips(i) = 0;
        case 3 % Max RPM
            % Set omega to max. omega
            omega(i) = halbach_wheel_parameters.m_omega;
            slips(i) = omega(i)*halbach_wheel_parameters.ro - v(i-1);
            f_thrust_wheel(i) = fx(slips(i), v(i-1), halbach_wheel_parameters);
    end
    
    % Cap torque
    % Note: In the deceleration phase the torque will be 0 as f_thrust_wheel(i) = 0 N/m
    torque(i) = halbach_wheel_parameters.ro*halbach_wheel_parameters.w*f_thrust_wheel(i);
    if torque(i) > halbach_wheel_parameters.m_torque
        torque(i) = halbach_wheel_parameters.m_torque;
        f_thrust_wheel(i) = torque(i)/(halbach_wheel_parameters.ro*halbach_wheel_parameters.w);
        slips(i) = fzero(@(x) (fx(x,v(i-1),halbach_wheel_parameters)-f_thrust_wheel(i)), [0 slips(i)]);
        omega(i) = (slips(i)+v(i-1))/halbach_wheel_parameters.ro;
    end
    
    % Cap angular velocity if the angular acceleration is too big (assume linear increase)
    alpha = abs(omega(i)-omega(i-1))/dt; % Calculate angular acceleration in change of omega
    if alpha > halbach_wheel_parameters.m_alpha
        switch phase
            case 1 % Acceleration    
                omega(i) = omega(i-1) + halbach_wheel_parameters.m_alpha*dt;
                f_thrust_wheel(i) = fx(slips(i), v(i-1), halbach_wheel_parameters);
                slips(i) = halbach_wheel_parameters.ro*omega(i) - v(i-1);
            case 2 % Deceleration
                f_thrust_wheel(i) = 0;
                omega(i) = 0;
                slips(i) = 0;
        end
    end

    
    % Calculate total x forces
    f_x_pod(i) = f_thrust_wheel(i)*halbach_wheel_parameters.w*n_wheel;

    % Calculate lateral force from Halbach wheel and total y force
    f_lat_wheel(i) = fy(slips(i), v(i-1), halbach_wheel_parameters);
    f_y_pod(i) = f_lat_wheel(i)*halbach_wheel_parameters.w*n_wheel;
    
    % Calculate acceleration, velocity and distance
    a(i) = f_x_pod(i)/halbach_wheel_parameters.M;
    
    % EmBrake braking
    switch phase
        case 2
            f_lat_wheel(i) = 0;
            f_y_pod(i) = 0;
            a(i) = -a_embrakes;
    end
    
    % Calculate trajectory
    v(i) = v(i-1) + dt*a(i);
    distance(i) = distance(i-1)+dt*v(i);
    
    % Calculate lateral torque, power and efficiency
    torque_lat(i) = halbach_wheel_parameters.ro*halbach_wheel_parameters.w*f_lat_wheel(i);
    power(i) = f_x_pod(i)*v(i); % power output = force * velocity
    power_loss(i) = n_wheel*halbach_wheel_parameters.w*pl(slips(i), v(i), halbach_wheel_parameters);
    power_input(i) = power(i)+power_loss(i); % ignoring inertia
    
    % When using embrakes we set the power input to 0
    switch phase
        case 2
            power_input(i) = 0;
    end
            
    efficiency(i) = power(i)/power_input(i);

end

