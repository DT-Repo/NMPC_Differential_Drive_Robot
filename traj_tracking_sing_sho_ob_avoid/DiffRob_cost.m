function v = DiffRob_cost(x,Ts,Np,th,obs,n_obs,st_ref,st0,path)
% Retrieve task's data
x_goal             =    st_ref(1,1);
y_goal             =    st_ref(2,1);
u_in               =       [x(1:Np,1)';
                        x(Np+1:2*Np,1)'];
                    
% Extract information about obstacles
xc              =       ones(Np+1,n_obs);
yc              =       ones(Np+1,n_obs);
r               =       ones(Np+1,n_obs);
safety_dist     =       0.3485/2+0.1;         %safety distance from obstacles

for i=1:n_obs
    xc(:,i)      =  xc(:,i).*obs(i,1);
    yc(:,i)      =  yc(:,i).*obs(i,2);
    r(:,i)       =  r(:,i).*obs(i,3) + safety_dist;
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
traj_ref = [path(:,1);path(:,2)];
%% Compute path constraints h(x)
h=zeros((Np+1)*n_obs,1);
for i=1:n_obs
    h((i-1)*Np+i:i*Np+i,1) = sqrt((Y_sim-yc(:,i)).^2+(X_sim-xc(:,i)).^2)-r(:,i);
end
h=[h;X_sim;Y_sim;(10-0.1742)-X_sim;(8-0.1742)-Y_sim];


%% Compute cost function f(x)
 delta_diff =              [traj_ref(1:Np+1,1)-traj_sim(1:Np+1,1);     
                         traj_ref(Np+2:2*Np+2,1)-traj_sim(Np+2:2*Np+2,1)];
ctrl_effort     =   [u_in(1,2:end)'-u_in(1,1:end-1)';u_in(2,2:end)'-u_in(2,1:end-1)'];  % variation 
%ctrl_effort     =   [u_in(1,:)';u_in(2,:)'];                   no variation
Q = 5*eye(2*Np+2,2*Np+2);
R               =   5*eye(2*Np-2,2*Np-2);
f=(delta_diff'*Q*delta_diff)+(ctrl_effort'*R*ctrl_effort);
%% Stack cost and constraints
v           =   [f;h];

end


