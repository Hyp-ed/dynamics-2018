function fx = fx(slip,vt,parameters)
% FX            Calculates thrust force as a function of slip and velocity
% Inputs:
%   slip        Slip ratio
%   vt          Translational Velocity
%   parameters  A struct array containing Halbach wheel parameters
%               See 'importHalbachWheelParameters.m'
% Output:
%   fx          Thrust force

%Define variables
if nargin > 1
    parameters.s = slip;
    parameters.vt = vt;
end

fx = calc_Final(parameters,[1 0 0]);

end