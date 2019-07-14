function optimalSlip = calc_optimalSlip(velocity,os_coefficients)
% CALC_OPTIMALSLIP	Calculates the optimal slip for a given velocity.
    optimalSlip = polyval(os_coefficients.optimalSlipCoefficients, velocity);
end