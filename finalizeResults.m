function [ result ] = finalizeResults(max,t,d,v,a,rpm,torque,torque_lat,f_thrust_wheel,f_lat_wheel,f_x_pod,f_y_pod,power,power_loss,power_input,eff,slips,omega)
% finalizeResults  Truncates trajectory arrays and creates a results structure
% Inputs:
%   max            Last used index in result arrays 
%                  (will be truncated up to this index)
%   t              Time array
%   d              Distance travelled over time
%   v              Velocity of pod over time
%   a              Acceleration of pod over time
%   rpm            RPM of Halbach wheels over time
%   torque         Torque on Halbach wheels over time
%   f_thrust_wheel Single Halbach wheel thrust force over time
%   f_lat_wheel    Single lateral Halbach wheel force over time
%   f_x_pod        Total x force over time
%   f_y_pod        Total y force over time
%   power          Power over time
%   eff            Efficiency over time
%   slips          Slips over time

% Output:
%   [ result ]     Result structure
% @author          Rafael Anderka
%                  HypED, 03/11/2018
% Modified:        -

    % Create result structure while truncating each array up to 'max'
    result = struct;
    result.time = t(1:max);
    result.velocity = v(1:max);
    result.acceleration = a(1:max);
    result.distance = d(1:max);
    result.rpm = rpm(1:max);
    result.torque = torque(1:max);
    result.torque_lat = torque_lat(1:max);
    result.wheel_thrust = f_thrust_wheel(1:max);
    result.wheel_lat = f_lat_wheel(1:max);
    result.pod_x = f_x_pod(1:max);
    result.pod_y = f_y_pod(1:max);
    result.power = power(1:max);
    result.power_loss = power_loss(1:max);
    result.power_input = power_input(1:max);
    result.efficiency = eff(1:max);
    result.slips = slips(1:max);
    result.omega = omega(1:max);


end

