function pl = pl(slip,vt,parameters)
% PL            Calculates power loss as a function of slip and velocity
% Inputs:
%   slip        Slip ratio
%   vt          Translational Velocity
%   parameters  A struct array containing Halbach wheel parameters
%               See 'importHalbachWheelParameters.m'
% Output:
%   pl          Power loss

%Define variables
if nargin > 1
    parameters.s = slip;
    parameters.vt = vt;
end

pl = calc_Final(parameters,[0 0 1]);

end

