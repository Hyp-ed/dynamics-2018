function pl = pl(slip,vt,parameters)
% PL            Calculates power loss as a function of slip and velocity
% Inputs:
%   slip        Slip ratio
%   vt          Translational Velocity
%   parameters  A struct array containing Halbach wheel parameters
%               See 'importHalbachWheelParameters.m'
% Output:
%   pl          Power loss

    % Write slip and current speed into paramteres structure
    parameters.s = slip;
    parameters.vt = vt;

    % Calculate power loss
    pl = calc_Final(parameters,[0 0 1]);

end

