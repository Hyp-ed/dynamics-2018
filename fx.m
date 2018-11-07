function fx = fx(slip,vt,parameters)
% FX            Calculates thrust force as a function of slip and velocity
% Inputs:
%   slip        Slip ratio
%   vt          Translational Velocity
%   parameters  A struct array containing Halbach wheel parameters
%               See 'importHalbachWheelParameters.m'
% Output:
%   fx          Thrust force

    % Write slip and current speed into paramteres structure
    parameters.s = slip;
    parameters.vt = vt;

    % Calculate slip and corresponding driving force
    fx = calc_Final(parameters,[1 0 0]);

end