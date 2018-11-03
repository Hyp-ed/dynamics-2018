function [lift, drag, height] = FDP_helper_fn_lev_forces(vt, halbach_array_parameters)
% FDP_TRAJECTORY_LEV_FORCES     A wrapper for the FDP_sim_pod_trajectory
%                               script to calculate lift and drag forces 
%                               from levitation skis
% Inputs:
%   vt                          Translational velocity
%   halbach_array_parameters    Struct array containing halbach array
%                               parameters
% Outputs:
%   lift                        Lift force from lev ski
%   drag                        Drag force from lev ski

    skin_depth = lev_calc_skin_depth(vt, halbach_array_parameters);
    [height] = lev_height(vt, halbach_array_parameters, skin_depth);
    
    if height  <= 0.007 % roller height is the minimum height
        height = 0.007;
    end
    
    [lift, drag] = lev_forces(vt, halbach_array_parameters, skin_depth, height);
end

