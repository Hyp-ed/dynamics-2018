function omegae = calc_omegae( slip, vt, ro, P )
% omegae    Calculates constant omega e

    omegae = -(slip+vt).*P./ro;

end

