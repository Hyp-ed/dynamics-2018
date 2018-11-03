function [ Lift, Drag ] = lev_forces( v, Input, skinDepth, Air_gap )
% FORCES        Calculate lift and drag forces of the levitation ski
% Inputs:
%   v           
%   Input       Struct array containing Halbach array parameters
%               See 'importHalbachArrayParameters
%   skinDepth
%   air_gap
% Outputs:
%   lift
%   drag
% @author       Ivan Chan, Head of Simulation Team, HypED, 25/01/2017

    %Calculate the "Max" Lift Force (in theory)
    Max_Lift_Force = transpose((Input.AreaLev*Input.B0_lev^2/Input.mu0)*exp(-2*Input.k*Air_gap-Input.k*Input.deltaC));
    
    %Applying the constants to caluate the actual lift force
    Lift = Max_Lift_Force.*((sqrt(1+Input.k.^4*skinDepth.^4*1/4)-Input.k^2*skinDepth.^2*1/2).^(3/2))./(Input.k*skinDepth + (sqrt(1+Input.k^4*skinDepth.^4*1/4)+Input.k^2*skinDepth.^2*1/2).^(3/2));
    
    %Applying the lift and drag ratio to calculate the drag force
    Drag = Lift.*((Input.k.*skinDepth)./(sqrt(1+Input.k^4*skinDepth.^4/4)-Input.k^2*skinDepth.^2/2).^(1/2));
    
    if v(1) == 0
        Lift(1:length(Air_gap),1) = 0; % manually setting the force when v = 0 m/s (ie. there is no induction)
        Drag(1:length(Air_gap),1) = 0; % manually setting the force when v = 0 m/s (ie. there is no induction)
    end
    
end