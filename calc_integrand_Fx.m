function FxInt = calc_integrand_Fx( xi, omegae, C, Input )
% CALC_INTEGRAND_FX  Computes integrand from the thrust force equation (35).
% Inputs:
%   xi      Fourier transform variable
%   omegae  Electrical angular velocity, see calc_omega.m
%   C       Constant, see calc_C.m
%   Input   Struct array containing Halbach wheel parameters
%           See 'importHalbachWheelParameters.m'
% %   incl_w  If 0, do not multiply by width (gives force per unit width)
% %           Otherwise, multiply integrand by width
% Output:
%   FxInt   Integrand of the thrust force from equation (35), units N/m
% Equations from paper:
%           Nirmal Paudel and Jonathan Z. Bird, 'General 2-D Steady-State 
%           Force and Power Equations for a Traveling Time-Varying Magnetic 
%           Source Above a Conductive Plate', IEEE TRANSACTIONS ON 
%           MAGNETICS, VOL. 48, NO. 1, JANUARY 2012
% Source code by Nirmal Paudel. Refactored by Isabella Chan, HypED 2017

    % Read halbach wheel parameters
    b = Input.t;
    y = b; % on the plate's surface
    g = Input.g;
    ro = Input.ro;
    mu0 = Input.mu0;
    % width = Input.w;
    sigma_ = Input.sigma_;
    vt = Input.vt;
    P = Input.P;
    
    % Gamma: equation (17)
    r = sqrt(xi.^2 + 1i.*mu0.*sigma_.*(omegae+vt.*xi));

    % T: equation (32)
    denominator = (exp(r.*b).*(r+xi).^2 - exp(-r.*b).*(r-xi).^2);
    T = ((r+xi).*exp(r.*y) + (r-xi).*exp(-r.*y))./denominator;
    dT = r.*((r+xi).*exp(r.*y)-(r-xi).*exp(-r.*y))./denominator;

    % Source field (an estimation for pole pair = 4)
    % Bxy = pi./6.*C.*xi.^4.*exp(-xi.*(g+ro)).*heaviside(xi); % Bxy field (56)
    % Bx = dT.*Bxy;
    % By = -1i.*xi.*T.*Bxy;
    
    % at y = b (for arbitrary number of pole pairs)
    Bsx = ((-1i).^P).*(2/factorial(P)).*C.*pi.*(xi.^P).*exp(-xi.*(g+ro)).*heaviside(xi);
    Bsy = ((-1i).^(P+1)).*(2/factorial(P)).*C.*pi.*(xi.^P).*exp(-xi.*(g+ro)).*heaviside(xi);
    Bsxy = Bsx+1j.*Bsy; % equation (25)
    By = -1i.*xi.*T.*Bsxy; % equation (33)
    Bx = dT.*Bsxy; % equation (34)
    % Integrands
    k = 1./(4.*pi.*mu0);

    FxInt = k.*real(conj(Bx).*By); % (N/m)
    
end

