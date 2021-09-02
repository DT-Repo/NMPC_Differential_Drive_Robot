% Constrained Numerical Optimization for Estimation and Control Project
%
% Non linear MPC - Multiple Shooting for a differential-drive robot 
% with obstacle avoidance.

clear all
close all
clc

%% Model parameters
base        =       0.235;      %distance between two wheels [m]
r           =       0.036;      %radius robot's wheels [m]
rob_diam    =       0.3485;     %robot's size [m]
th          =       [r;base];

%% Envinroment parameters
n_obs   =       12;           %number of obstacles           
h_map   =       8;           %height of the map
w_map   =       10;          %width of the map

%% Generate map
%generate a map with random obstacles shaped as circles
%[x_,y_,xc,yc,rad]   =     generate_map(w_map,h_map,n_obs);     

load('..\single_shooting_obs_avoid\easy_map\x_test.mat');
load('..\single_shooting_obs_avoid\easy_map\y_test.mat');
load('..\single_shooting_obs_avoid\easy_map\xc_test.mat');
load('..\single_shooting_obs_avoid\easy_map\yc_test.mat');
load('..\single_shooting_obs_avoid\easy_map\rad_test.mat');

% hard map contains n_obs=15
% load('..\single_shooting_obs_avoid\easy_map\x_test2.mat');
% load('..\single_shooting_obs_avoid\easy_map\y_test2.mat');
% load('..\single_shooting_obs_avoid\easy_map\xc_test2.mat');
% load('..\single_shooting_obs_avoid\easy_map\yc_test2.mat');
% load('..\single_shooting_obs_avoid\easy_map\rad_test2.mat');


obs               =     [xc,yc,rad];
%% Define start and goal
start = [rob_diam/2;rob_diam/2;0];        
goal = [8;7;0];

%% FHOCP parameters - Single Shooting
Ts      =       0.5;                % seconds, input sampling period
Tend    =       50;                 % seconds, terminal time
Np      =       8;            % prediction horizon

%% Initialize optimization variables
x0      =       [ zeros(Np,1);      % inputs: v(m/s) and omega(rad/s)
                  zeros(Np,1);
                  zeros(Np,1);      %states: x [m] and y [m] and theta [rad]
                  zeros(Np,1);
                  zeros(Np,1)];     
                    
              
%% Linear equality constraint parameters
A               =   [];
b               =   [];
                       
%% Constraints
%Bounds on input variables
omega_max   =       2.34; %[rad/s]
v_max       =       0.55; %[m/s] 
C           =       [-eye(5*Np);
                    eye(5*Np)];
d           =       [ones(Np,1)*-v_max;
                    ones(Np,1)*-omega_max;
                    ones(Np,1)*-10+rob_diam/2;
                    ones(Np,1)*-8+rob_diam/2;
                    ones(Np,1)*-pi*2/3;
                    ones(Np,1)*-v_max;
                    ones(Np,1)*-omega_max;
                    ones(Np,1)*0+rob_diam/2;
                    ones(Np,1)*0+rob_diam/2;
                    ones(Np,1)*-pi*2/3];
        
q           =        n_obs*(Np+1);            % Number of nonlinear inequality constraints

%% Setup Solver options

myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'GN';
%myoptions.Hessmethod  	=	'BFGS';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-8;
myoptions.ls_beta       =	0.5;
myoptions.ls_c          =	.1;
myoptions.ls_nitermax   =	1e2;
myoptions.nitermax      =	1e3;
myoptions.xsequence     =	'off';

%% Define conditions to shift Horizon Window and simulations
t0       =       0;
t(1)     =       t0;
st_0      =      start;    % initial states
u0       =       x0(1:Np*2,1);       % two control inputs
st_ref    =      goal;     % Reference states

n_iter   =       0;
u_cl     =       [];       % vector to store optimal actions (Receding Horizon)

xx(:,1) =        st_0;       % vectors to store history of states and
xx1     =        [];        % plot results

x_opt =zeros(Np*3,1);

%% Non Linear MPC Strategy
mpc_loop = tic;
while(norm((st_0-st_ref),2) > 1e-1 && n_iter < Tend / Ts)
    x0 = [u0;x_opt];
    myoptions.GN_funF = @(x)DiffRob_cost(x,Ts,Np,th,obs,n_obs,goal,st_0);
    % Solve FHOCP
    [xstar,fxstar,niter,exitflag,xsequence] = myfmincon(@(x)DiffRob_cost(x,Ts,Np,th,obs,n_obs,st_ref,st_0),x0,[],[],C,d,0,q,myoptions);
    u = xstar(1:2*Np,1);
    x_opt = [xstar(2*Np+1:3*Np,1)';xstar(3*Np+1:4*Np,1)';xstar(4*Np+1:5*Np,1)'];
    
    % Simulate trajectory for visualization
    u_star               =       [xstar(1:Np,1)';       
                        xstar(Np+1:2*Np,1)'];
    xi_opt=zeros(3,Np+1);                   
    xi_opt(:,1)=st_0;
    for ind=1:Np
        xidot                 =   diff_drive(0,xi_opt(:,ind),u_star,th);
        xi_opt(:,ind+1)       =   xi_opt(:,ind)+Ts*xidot;
    end

    u_mpc = [xstar(1,1);xstar(Np+1,1)];

    xx1(:,1:3,n_iter+1)= full(xi_opt)';   % store trajectory for visualization
    u_cl= [u_cl ; u_mpc(1,1) u_mpc(2,1)]; %store u_mpc for visualizaton 
    t(n_iter+1) = t0;

    % Init next Horizon window
    [t0, st_0, u0] = move_horizon(Ts, t0, st_0, u, Np,th); 

    xx(:,n_iter+2) = st_0; %store state after executed u_mpc for visualization

    % Shift trajectory to initialize the next step
    x_opt=[x_opt(1,:)';x_opt(2,:)';x_opt(3,:)'];
    x_opt=[x_opt(:,2:end);x_opt(:,end)];
    % Set next step
    n_iter
    n_iter = n_iter + 1;
end
mpc_time = toc(mpc_loop);
error = norm(st_0(1:2,1)-st_ref(1:2,1),2)
average_mpc_time = mpc_time/(n_iter+1)

%% Show results
Robot_traj (t,xx,xx1,u_cl,st_ref,Np,rob_diam,x_,y_,w_map,h_map,th) % a drawing function