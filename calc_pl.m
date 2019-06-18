function pl = pl(slip,vt,pl_lookup_table,halbach_wheel_parameters)
% PL            Calculates power loss as a function of slip and velocity by bilinearly interpolating a power loss lookup table
% Inputs:
%   slip            Absolute slip
%   vt              Translational Velocity
%   pl_lookup_table A COMSOL-generated power loss lookup table
% Output:
%   pl              Power loss per wheel

    % Slip indices (x-axis)
    i_s = slip / pl_lookup_table.s_step + 1; % x
    i_s_min = floor(i_s); % x1
    i_s_max = i_s_min + 1; % x2
    
    % Velocity indices (y-axis)
    i_v = vt / pl_lookup_table.v_step + 1; % y
    i_v_min = floor(i_v); % y1
    i_v_max = i_v_min + 1; % y2
    
    % Bilinear interpolation
    pl1 = (i_s_max - i_s) / (i_s_max - i_s_min) * pl_lookup_table.powerLosses(i_v_min, i_s_min) + (i_s - i_s_min) / (i_s_max - i_s_min) * pl_lookup_table.powerLosses(i_v_min, i_s_max); % Interpolate along x-axis for y1
    pl2 = (i_s_max - i_s) / (i_s_max - i_s_min) * pl_lookup_table.powerLosses(i_v_max, i_s_min) + (i_s - i_s_min) / (i_s_max - i_s_min) * pl_lookup_table.powerLosses(i_v_max, i_s_max); % Interpolate along x-axis for y2
    pl = (i_v_max - i_v) / (i_v_max - i_v_min) * pl1 + (i_v - i_v_min) / (i_v_max - i_v_min) * pl2; % Interpolate along y-axis between (x, y1) and (x, y2)
    pl = pl / 2 * halbach_wheel_parameters.w; % Divide by two since the lookup table is for wheel pairs and multiply by the magnet thickness

end

