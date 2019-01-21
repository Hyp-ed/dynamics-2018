function C = calc_C( Br, P, mur, ro, ri)
% calc_C    Calculates constant C from equation (47)
% Inputs:
%   Br      remanence of the magnet
%   P       number of pole pairs
%   mur     relative permeability of the magnet.
%   ro      outer rotor radius
%   ri      inner rotor radius
% Equations from paper:
%           Nirmal Paudel and Jonathan Z. Bird, 'General 2-D Steady-State 
%           Force and Power Equations for a Traveling Time-Varying Magnetic 
%           Source Above a Conductive Plate', IEEE TRANSACTIONS ON 
%           MAGNETICS, VOL. 48, NO. 1, JANUARY 2012

    k = -(2.*Br.*P)./(P+1);
    component1 = ((1+mur).*ro.^(2.*P)).*(ro.^(P+1) - ri.^(P+1));
    component2 = ((1-mur).^2 .*ri.^(2.*P) - (1+mur).^2 .* ro.^(2.*P));
    
    C = k.*component1./component2;
    
end

