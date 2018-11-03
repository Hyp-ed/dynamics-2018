function [ Height ] = lev_height(v,Input,skinDepth)
% LEV_HEIGHT    Determines the equilibrium heigh of the pod as a function
%               of its velocity
% @author       Emil Hansen, Head of Dynamics, HypED, 28/01/2017
    
   % Calculate the wanted lift force (ie. weigth of the pod)
   Lift_Equilibrium = Input.M * 9.81;

   % Calculate the "Max" Lift Force (in theory)
   a = (sqrt(1 + Input.k^4*skinDepth.^4*1/4) - Input.k^2*skinDepth.^2*1/2).^(3/2);
   b = Input.k*skinDepth + (sqrt(1+ Input.k^4*skinDepth.^4*1/4) + Input.k^2*skinDepth.^2*1/2).^(3/2);
   Fymax = Lift_Equilibrium.*b./a;
   
   % Reverse calulation to evaluate the lev Height
   Height = (((log(Fymax*Input.mu0/(Input.B0_lev^2*Input.AreaLev))/-Input.k)-Input.deltaC)/2);
   
   for i = 1:length(v) %Loop to adjust the 'levitation height' at low speed
       
       if v(i) == 0
           Height(i)= Input.rollerHeight;% manually adjust the levitation height when v = 0m/s
       elseif Height(i) <Input.rollerHeight
           Height(i) = Input.rollerHeight;
       end
       
   end
   
end
