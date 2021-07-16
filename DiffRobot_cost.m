function v = DiffRobot_cost(x,Ts,Np,th,obs,n_obs,x_goal,y_goal)

% Function that computes the trajectory of the differential-drive robot
%  and returns the distance X covered (cost function) and the constraints 


%% Build vector of inputs
%(Np-1)*Ts
t_in            =       [0:Ts:20-Ts]';
xi0             =       [x(1,1);x(2,1);x(3,1)];
u_in            =       [x(4:Np+3,1)';
                        x(Np+4:2*Np+3,1)'];
xc              =       ones(Np,n_obs);
yc              =       ones(Np,n_obs);
r               =       ones(Np,n_obs);
n_obs           =       n_obs;
safety_dist     =       0.5;         %safety distance from obstacles
scale_factor    =       1;
for i=1:n_obs
    xc(:,i)      =  round(xc(:,i).*obs(i,1),1)*scale_factor;
    yc(:,i)      =  round(yc(:,i).*obs(i,2),1)*scale_factor;
    r(:,i)       =  round(r(:,i).*obs(i,3)*scale_factor) + safety_dist;
    
end

assignin('base','xi0',xi0);
assignin('base','t_in',t_in);
assignin('base','u_in',u_in);

%% Run simulation with FFD
time_FFD    =   [0:0.1:(Np-1)*Ts];
Nblock      =   Ts/0.1;
Nsim_FFD    =   length(time_FFD);
%states
xi_sim      =   zeros(3,Np+1);
xi_sim(:,1) =   xi0;
 
for ind=2:Nsim_FFD
    u                   =   u_in(:,1+floor(time_FFD(ind)));
    xidot               =   diff_drive(0,xi_sim(:,ind-1),u,th);
    xi_sim(:,ind)       =   xi_sim(:,ind-1)+Ts/Nblock*xidot;
end


X_sim       =   xi_sim(1,1:Nblock:end)';
Y_sim       =   xi_sim(2,1:Nblock:end)';


traj_sim    =   [X_sim;Y_sim];
traj_ref    =   [ones(size(X_sim))*x_goal;ones(size(Y_sim))*y_goal];
%% Compute path constraints h(x)
h=zeros(Np*n_obs,1);
for i=1:n_obs
    h((i-1)*Np+1:i*Np,1) = (Y_sim-yc(:,i)).^2+(X_sim-xc(:,i)).^2-r(:,i).^2;
end

%% Compute cost function f(x)
delta_diff      =   [traj_ref(1:Np-1,1)-traj_sim(1:Np-1,1);     
                        traj_ref(Np+1:2*Np-1,1)-traj_sim(Np+1:2*Np-1,1)];
delta_end       =   [traj_ref(Np,1)-traj_sim(Np,1);
                   traj_ref(2*Np,1)-traj_sim(2*Np,1)];
ctrl_effort     =   [u_in(1,:)';u_in(2,:)'];
Q               =   eye(2*Np-2,2*Np-2);
S               =   eye(2,2);
R               =   eye(2*Np,2*Np);

f               =   (delta_diff'*Q*delta_diff)+(delta_end'*S*delta_end)+(ctrl_effort'*R*ctrl_effort);

%% Stack cost and constraints
v           =   [f;h];

end

