function [v,a,distance,omega,torque,torque_lat,torque_motor,power,power_loss,power_input,efficiency,slips,f_thrust_wheel,f_lat_wheel,f_x_pod,f_y_pod] = calc_main(phase,i,dt,n_wheel,v,a,distance,omega,torque,torque_lat,torque_motor,power,power_loss,power_input,efficiency,slips,f_thrust_wheel,f_lat_wheel,f_x_pod,f_y_pod,halbach_wheel_parameters, a_embrakes)
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
            
            % Calculate torques
            torque(i) = halbach_wheel_parameters.ro*halbach_wheel_parameters.w*f_thrust_wheel(i);
            power_loss(i) = n_wheel*halbach_wheel_parameters.w*pl(slips(i), v(i-1), halbach_wheel_parameters);
            torque_motor(i) = torque(i)+(power_loss(i)/(n_wheel*omega(i)));
            
            % Cap motor torque
            if torque_motor(i) > halbach_wheel_parameters.m_torque
                torque_motor(i) = halbach_wheel_parameters.m_torque;
                slips(i) = fzero(@(x) ((fx(x, v(i-1), halbach_wheel_parameters) * halbach_wheel_parameters.w * halbach_wheel_parameters.ro) + (pl(x, v(i-1), halbach_wheel_parameters) * halbach_wheel_parameters.w / ((x+v(i-1))/halbach_wheel_parameters.ro)) - torque_motor(i)), [0.1, slips(i)]);
                f_thrust_wheel(i) = fx(slips(i), v(i-1), halbach_wheel_parameters);
                power_loss(i) = n_wheel*halbach_wheel_parameters.w*pl(slips(i), v(i-1), halbach_wheel_parameters);
                omega(i) = (slips(i)+v(i-1))/halbach_wheel_parameters.ro;
                torque(i) = halbach_wheel_parameters.ro*halbach_wheel_parameters.w*f_thrust_wheel(i);
            end
        case 2 % Deceleration using EmBrakes
            omega(i) = 0;
            f_thrust_wheel(i) = 0;
            slips(i) = 0;
            power_loss(i) = 0;
            torque(i) = 0;
            torque_motor(i) = 0;
        case 3 % Max RPM
            % Set omega to max. omega
            omega(i) = halbach_wheel_parameters.m_omega;
            slips(i) = omega(i)*halbach_wheel_parameters.ro - v(i-1);
            f_thrust_wheel(i) = fx(slips(i), v(i-1), halbach_wheel_parameters);
            torque(i) = halbach_wheel_parameters.ro*halbach_wheel_parameters.w*f_thrust_wheel(i);
            power_loss(i) = n_wheel*halbach_wheel_parameters.w*pl(slips(i), v(i-1), halbach_wheel_parameters);
            torque_motor(i) = torque(i)+(power_loss(i)/(n_wheel*omega(i)));
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
    power_input(i) = power(i)+power_loss(i); % ignoring inertia
    
    % When using embrakes we set the power input and motor torque to 0
    switch phase
        case 2
            power_input(i) = 0;
            torque_motor(i) = 0;
    end
            
    efficiency(i) = power(i)/power_input(i);

end

