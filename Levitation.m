% Levitation    Calculate the lift and the drag of the pod as a function of
%               velocity and air gap.
% @author       Emil Hansen, Ivan Chan
%               HypEd, 25/01/2017

clear; clc;
  
% Parameter=Parameters(); % Import the parameters
Parameter = importHalbachArrayParameters();
v = (0:1:140); % Setting up a range of velocity
skinDepth = lev_calc_skin_depth(v,Parameter);
h = (0.001:0.001:0.5); % Setting up a range of levitation height

lev_h = lev_height(v,Parameter,skinDepth);%%Calling the function for Equilibrium_Height
[Lift_equ,Drag_equ] = lev_forces_equilibrium(v, Parameter,skinDepth, lev_h);
Lift_equ=Lift_equ(1,:);
Drag_equ=Drag_equ(1,:);

[Lift,Drag] = lev_forces(v, Parameter,skinDepth, h); % Calling the function for Lift_Force

% 2D graph for lift force in terms of each unit of velocity and range of levitation
% height
plot(v,transpose(Lift)); title('Lift Force vs. Velocity'); xlabel('Velocity /m/s'); ylabel('Lift Force /N');
grid on

% 3D graph for lift force in terms of each unit of velocity and range of levitation
% height
figure;
surf(h,v,transpose(Lift));title('Lift Force vs Levitation Height vs Velocity');xlabel('Levitation Height /m'); ylabel('Velocity /m/s'); zlabel('Lift Force /N');
grid on

% 2D graph shows the equilibrium levitation height against each unit of velocity Equilibrium Height
figure;
plot(v,lev_h); title('Equilibrium Height vs Velocity'); xlabel('Velocity /m/s'); ylabel('Equilibrium Height /m');
grid on

% 2D graph for drag force due to levitation in terms of each unit of velocity and range of levitation
% height
figure;
plot(v,transpose(Drag)); title('Drag Force (Levitation) vs Velocity'); xlabel('Velocity /m/s'); ylabel('Drag Force (Levitation) /N');
grid on

% 3D graph for drag force due to levitation in terms of each unit of velocity and range of levitation
% height
figure;
surf(h,v,transpose(Drag));title('Drag Force (Levitation) vs Airgap vs Velocity');xlabel('Levitation Height /m'); ylabel('Velocity /m/s'); zlabel('Drag Force (Levitation) /N');
grid on

% 2D graph shows the drag force and lift force due to levitation at equilibrium height against each unit of velocity 
plot(v,Drag_equ); 
title('Lift and Drag Force (Levitation) vs Velocity'); xlabel('Velocity /m/s'); ylabel('Forces (Levitation) /N');
grid on
hold on
plot(v,Lift_equ);
legend('Drag Force','Lift Force');

