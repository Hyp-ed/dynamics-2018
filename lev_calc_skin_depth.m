function [skinDepth] = lev_calc_skin_depth(v,Input)
    skinDepth = sqrt(2*Input.rho./(Input.mu0*Input.k*v));
end 