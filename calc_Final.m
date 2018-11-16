function [ varargout ] = calc_Final(parameters, whichValues)
% CALC_FINAL    Calculates the final values for the thrust and lateral forces,
%               and the power loss ratio
% Inputs:        
%   parameters  Struct array containing all the necessary parameters. 
%               See 'importHalbachWheelParameters.m'
%   whichValues Array which tells the function which values to compute, so
%               that calc_Final can be used to calculate just the driving
%               force, lateral force and power loss if necessary. Its default
%               value is set to [1 1 1], i.e., all results are computed.
% Outputs:
%   varargout   Forces computed are driving force (Fx), lateral force (Fy) and
%               power loss (PLoss). Depending on the values in whichValues,
%               either of the above can be returned through varargout.
% Author:
% Ivan Chan, Head of HypED Simulation Team, UoEdinburgh. 05/10/2017

    %Set default value of which values
    if nargin == 1
        whichValues = [1 1 1];
    end

    % Begin calculation
    C = calc_C(parameters.Br, parameters.P, parameters.mur, parameters.ro, parameters.ri);
    omegae = calc_omegae(parameters.s, parameters.vt, parameters.ro, parameters.P);

    %Decide which values should be computed and import them in varargout. 
    k = 1;
    if whichValues(1) == 1
        Fx = quadgk(@(xi)calc_integrand_Fx(xi, omegae, C, parameters), -250, 250);
        varargout{k} = Fx;
        k=k+1;
    end
    
    if whichValues(2) == 1
        Fy = quadgk(@(xi)calc_integrand_Fy(xi, omegae, C, parameters), -250, 250);
        varargout{k} = Fy;
        k =k+1;
    end

    if whichValues(3) == 1
        Fx = quadgk(@(xi)calc_integrand_Fx(xi, omegae, C, parameters), -250, 250);
        PT = quadgk(@(xi)calc_integrand_PT(xi, omegae, C, parameters), -250, 250);
        PLoss = PT - real(Fx).*parameters.vt;
        varargout{k} = PLoss;
    end

end