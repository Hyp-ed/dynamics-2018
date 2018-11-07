function fy = fy(slip,vt,parameters)
% FY            Calculates lateral force as a function of slip and velocity
% Inputs:
%   slip        Slip ratio
%   vt          Translational Velocity
%   parameters  A struct array containing Halbach wheel parameters
%               See 'importHalbachWheelParameters.m'
% Output:
%   fy          Lateral force

% Write slip and current speed into paramteres structure
parameters.s = slip;
parameters.vt = vt; 

% Calculate lateral force
fy = calc_Final(parameters, [0 1 0]);

end