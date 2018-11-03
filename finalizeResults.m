function [ result ] = finalizeResults(max,t,d,v,a,rpm,torque,f_drag_wheel,f_drag_lev,f_lift_lev,f_drag_pod,f_lift_pod,power,eff,slips,f_lat_wheel,height)
% finalizeResults  Truncates trajectory arrays and creates a results structure
% Inputs:
%   max            Last used index in result arrays 
%                  (will be truncated up to this index)
%   t              Base time array
%   d              Distance over time array
%   v              Velocity over time array
%   a              Acceleration over time array
%   rpm            RPM over time array
%   torque         torque over time array
%   f_drag_wheel   Wheel drag force over time array
%   f_drag_lev     Lev drag force over time array
%   f_lift_lev     Lev lift force over time array
%   f_drag_pod     Pod drag force over time array
%   f_lift_pod     Pod lift force over time array
%   power          Power over time array
%   eff            Efficiency over time array
%   slips          Slips over time array
%   f_lat_wheel    Lat wheel force over time array
%   height         Height over time array
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
result.wheel_drag = f_drag_wheel(1:max);
result.lev_drag = f_drag_lev(1:max);
result.lev_lift = f_lift_lev(1:max);
result.pod_drag = f_drag_pod(1:max);
result.pod_lift = f_lift_pod(1:max);
result.power = power(1:max);
result.efficiency = eff(1:max);
result.slips = slips(1:max);
result.f_lat_wheel = f_lat_wheel(1:max);
result.lev_height = height(1:max);

end

