function [t0, x0, u0] = move_horizon(Ts, t0, st, u, Np,th)
% Implementation of Moving Block approach
u_mpc = [u(1,:);u(Np+1,:)];
xidot               =   diff_drive(0,st,u_mpc,th);
st = st + (Ts*xidot);
x0 = full(st);

t0 = t0 + Ts;
u0 = [u(2:size(u,1),:);u(size(u,1),:)];         %shift control action
end