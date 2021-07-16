function v = DiffRob_cost(x,Ts,Np,th,obs,n_obs,st_ref,st0)
% Retrieve task's data
x_goal             =    st_ref(1,1);
y_goal             =    st_ref(2,1);
u_in               =       [x(1:Np,1)';
                        x(Np+1:2*Np,1)'];
                    
% Extract information about obstacles
xc              =       ones(Np+1,n_obs);
yc              =       ones(Np+1,n_obs);
r               =       ones(Np+1,n_obs);
safety_dist     =       0.5;         %safety distance from obstacles

for i=1:n_obs
    xc(:,i)      =  round(xc(:,i).*obs(i,1),1);
    yc(:,i)      =  round(yc(:,i).*obs(i,2),1);
    r(:,i)       =  round(r(:,i).*obs(i,3)) + safety_dist;
end

%% Run simulation with FFD
% States Initialization
xi_sim      =   zeros(3,Np+1);
xi_sim(:,1) =   st0;

for ind=1:Np
    u                     =   u_in(:,ind);
    xidot                 =   diff_drive(0,xi_sim(:,ind),u,th);
    xi_sim(:,ind+1)       =   xi_sim(:,ind)+Ts*xidot;
end

X_sim       =   xi_sim(1,:)';
Y_sim       =   xi_sim(2,:)';

traj_sim    =   [X_sim;Y_sim];
traj_ref    =   [ones(size(X_sim))*x_goal;ones(size(Y_sim))*y_goal];
%% Compute path constraints h(x)
h=zeros((Np+1)*n_obs,1);
for i=1:n_obs
    h((i-1)*Np+i:i*Np+i,1) = (Y_sim-yc(:,i)).^2+(X_sim-xc(:,i)).^2-r(:,i).^2;
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


