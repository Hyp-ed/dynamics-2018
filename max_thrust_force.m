%% Function for calculating max thrust force 

halbach_wheel_parameters = importHalbachWheelParameters();
max_force = 0;
max_v = 0;
max_slip = 0;

for i=0.1:0.1:10
    v = i;
 parameters.s = 10;
 parameters.vt = 30;
[slips, f_thrust_wheel] = fminbnd(@(x) -fx(x,v,halbach_wheel_parameters),0,50);
if (-f_thrust_wheel > max_force)
    max_force = -f_thrust_wheel;
    max_v = v;
    max_slip = slips;
end
end

fprintf('max thrust is: %6.4f\n',max_force*halbach_wheel_parameters.w);
fprintf('slip is: %6.4f\n',max_slip);
fprintf('velocity is: %6.4f\n',max_v);

