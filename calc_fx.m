function fx = calc_fx(slip,vt,fx_lookup_table)
% CALC_FX           Calculates thrust force as a function of slip and velocity by bilinearly interpolating a thrust force lookup table
% Inputs:
%   slip            Absolute slip
%   vt              Translational Velocity
%   fx_lookup_table A COMSOL-generated thrust force lookup table
% Output:
%   fx              Thrust force per wheel
    
    % Mirror for negative slip
    s = sign(slip);
    slip = abs(slip);
    
    % Slip indices (x-axis)
    i_s = slip / fx_lookup_table.s_step + 1; % x
    i_s_min = floor(i_s); % x1
    i_s_max = i_s_min + 1; % x2
    
    % Velocity indices (y-axis)
    i_v = vt / fx_lookup_table.v_step + 1; % y
    i_v_min = floor(i_v); % y1
    i_v_max = i_v_min + 1; % y2
    
    % Bilinear interpolation
    fx1 = (i_s_max - i_s) / (i_s_max - i_s_min) * fx_lookup_table.forces(i_v_min, i_s_min) + (i_s - i_s_min) / (i_s_max - i_s_min) * fx_lookup_table.forces(i_v_min, i_s_max); % Interpolate along x-axis for y1
    fx2 = (i_s_max - i_s) / (i_s_max - i_s_min) * fx_lookup_table.forces(i_v_max, i_s_min) + (i_s - i_s_min) / (i_s_max - i_s_min) * fx_lookup_table.forces(i_v_max, i_s_max); % Interpolate along x-axis for y2
    fx = (i_v_max - i_v) / (i_v_max - i_v_min) * fx1 + (i_v - i_v_min) / (i_v_max - i_v_min) * fx2; % Interpolate along y-axis between (x, y1) and (x, y2)
    fx = fx / 2; % Divide by two since the lookup table is for wheel pairs
    fx = s * fx;
    
end