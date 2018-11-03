function fy = fy(slip,vt,parameters)
% FY            Calculates lift force as a function of slip and velocity
% Inputs:
%   slip        Slip ratio
%   vt          Translational Velocity
%   parameters  A struct array containing Halbach wheel parameters
%               See 'importHalbachWheelParameters.m'
% Output:
%   fy          Lift force

%Define variables
parameters.s = slip;
parameters.vt = vt; 

fy = calc_Final(parameters, [0 1 0]);

end