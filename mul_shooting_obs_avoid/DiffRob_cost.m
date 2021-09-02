function v = DiffRob_cost(x,Ts,Np,th,obs,n_obs,st_ref,st0)
% Retrieve task's data
x_goal             =    st_ref(1,1);
y_goal             =    st_ref(2,1);
th_goal            =    st_ref(3,1);

u_in               =       [x(1:Np,1)';
                        x(Np+1:2*Np,1)'];
                    
x_opt              =    x(2*Np+1:3*Np,1)';  
y_opt              =    x(3*Np+1:4*Np,1)';
th_opt             =    x(4*Np+1:5*Np,1)';

xi_opt             =    [x_opt;y_opt;th_opt];
% Extract information about obstacles
xc              =       ones(Np+1,n_obs);
yc              =       ones(Np+1,n_obs);
r               =       ones(Np+1,n_obs);
safety_dist     =       0.25;         %safety distance from obstacles

for i=1:n_obs
    xc(:,i)      =  xc(:,i).*obs(i,1);
    yc(:,i)      =  yc(:,i).*obs(i,2);
    r(:,i)       =  r(:,i).*obs(i,3) + safety_dist;
end

%% Run simulation with FFD
% States Initialization

xi_opt(:,1)              =   st0;
for ind=1:Np
    u                     =   u_in(:,ind);
    xidot                 =   diff_drive(0,xi_opt(:,ind),u,th);
    xi_opt(:,ind+1)       =   xi_opt(:,ind)+Ts*xidot;
    
end
x_opt = xi_opt(1,:)';
y_opt = xi_opt(2,:)';
th_opt = xi_opt(3,:)';

traj_ref    =   [ones(size(x_opt))*x_goal;ones(size(y_opt))*y_goal;ones(size(th_opt))*th_goal];
%% Compute path constraints h(x)
h=zeros((Np+1)*n_obs,1);
for i=1:n_obs
    h((i-1)*Np+i:i*Np+i,1) = sqrt((y_opt-yc(:,i)).^2+(x_opt-xc(:,i)).^2)-r(:,i);
end
xi_opt = [x_opt;y_opt;th_opt];
%% Compute cost function f(x)
delta_diff      =   [traj_ref(1:Np,1)-xi_opt(1:Np,1);     
                        traj_ref(Np+2:2*Np+1,1)-xi_opt(Np+2:2*Np+1,1)
                        ];
delta_end       =   [traj_ref(Np+1,1)-xi_opt(Np+1,1);
                   traj_ref(2*Np+2,1)-xi_opt(2*Np+2,1);
                   traj_ref(3*Np+3,1)-xi_opt(3*Np+3,1)];
ctrl_effort     =   [u_in(1,:)';u_in(2,:)'];
% BFGS Cost Function
Q               =   eye(2*Np,2*Np);
S               =   10^3*eye(3,3);
R               =   eye(2*Np,2*Np);
%f               =   (delta_diff'*Q*delta_diff)+(delta_end'*S*delta_end)+(ctrl_effort'*R*ctrl_effort);

% GN Cost Function
f               = [delta_diff;
                    delta_end.*5;
                    ctrl_effort.*2];
%% Stack cost and constraints
v           =   [f;h];

end


