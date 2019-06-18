function [interactionTorque] = calc_coggingTorque(angle,ct_lookup_table)
% CALC_COGGINGTORQUE Calculates interaction torque by linearly interpolating an interaction torque table

    % Angle indices
    i = mod(angle / ct_lookup_table.angle_step,length(ct_lookup_table.angles) - 1) + 1; % Repeat for 0 - 2pi
    i_min = floor(i);
    i_max = i_min + 1;
    
    % Linear interpolation
    interactionTorque = ct_lookup_table.torques(i_min) + (i - i_min) * (ct_lookup_table.torques(i_max) - ct_lookup_table.torques(i_min)) / (i_max - i_min);
end