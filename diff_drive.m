function [xidot] = diff_drive(t,xi,u,th)
%% Read parameters, states and inputs
% Parameter
r = th(1,1);
d = th(2,1);

% States
X = xi(1,1);
Y = xi(2,1);
theta = xi(3,1);

% Inputs
% omega_R = u(1,1);
% omega_L = u(2,1);
% 
% v      =     (omega_R+omega_L)/2*r;
% omega  =     (omega_R-omega_L)/d*r;
v = u(1,1);
omega = u(2,1);
% Model equations
xidot(1,1) = v*cos(theta);
xidot(2,1) = v*sin(theta);
xidot(3,1) = omega;
end


