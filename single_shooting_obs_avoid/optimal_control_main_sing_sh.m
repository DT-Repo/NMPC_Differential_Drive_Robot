% Constrained Numerical Optimization for Estimation and Control Project
%
% Non linear MPC - Single Shooting for a differential-drive robot 
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
load('.\easy_map\x_test.mat');
load('.\easy_map\y_test.mat');
load('.\easy_map\xc_test.mat');
load('.\easy_map\yc_test.mat');
load('.\easy_map\rad_test.mat');

%hard map contains n_obs=15
% load('.\hard_map\x_test2.mat');
% load('.\hard_map\y_test2.mat');
% load('.\hard_map\xc_test2.mat');
% load('.\hard_map\yc_test2.mat');
% load('.\hard_map\rad_test2.mat');
%[x_,y_,xc,yc,rad]   =     generate_map(w_map,h_map,n_obs);
obs               =     [xc,yc,rad];
%% Define start and goal
start = [rob_diam/2;rob_diam/2;0];        
goal = [8;7;0];

%% FHOCP parameters - Single Shooting
Ts      =       0.5;                % seconds, input sampling period
Tend    =       50;                 % seconds, terminal time
Np      =       8;            % prediction horizon

%% Initialize optimization variables
x0      =       [ zeros(Np,1);      % inputs: omega_L(rad/s) and omega_R(rad/s)
                  zeros(Np,1) ];     
                    
              
%% Linear equality constraint parameters
A               =   [];
b               =   [];
                       
%% Constraints
%Bounds on input variables
omega_max   =       2.34; %[rad/s]
v_max       =       0.55; %[m/s] 
C           =       [-eye(2*Np)
                    eye(2*Np)];
d           =       [ones(Np,1)*-v_max;
                    ones(Np,1)*-omega_max;
                    ones(Np,1)*-v_max;
                    ones(Np,1)*-omega_max ];
                         
q           =        n_obs*(Np+1)+(Np+1)*6;            % Number of nonlinear inequality constraints

%% Setup Solver options
myoptions               =   myoptimset;
%myoptions.Hessmethod  	=	'BFGS';
myoptions.Hessmethod  	=	'GN';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-8;
myoptions.ls_beta       =	0.5;
myoptions.ls_c          =	.1;
myoptions.ls_nitermax   =	1e2;
myoptions.nitermax      =	1e3;
myoptions.xsequence     =	'off';
% myoptions.display    	=	'Iter';
% Define conditions to shift Horizon Window 
t0       =       0;
t(1)     =       t0;
st_0      =      start;    % initial states
u0       =       x0;       % two control inputs 
st_ref    =      goal;     % Reference states

n_iter   =       0;
u_cl     =       [];       % vector to store optimal actions (Receding Horizon)

xx(:,1) =        st_0;       % vectors to store history of states and
xx1     =        [];        % plot results


%% Non Linear MPC Strategy
mpc_loop = tic;
while(norm((st_0(1:2,1)-st_ref(1:2,1)),2) > 1e-1 && n_iter < Tend / Ts)
    x0 = u0;
    myoptions.GN_funF = @(x)DiffRob_cost(x,Ts,Np,th,obs,n_obs,goal,st_0);
    % Solve FHOCP
    [xstar,fxstar,niter,exitflag,xsequence] = myfmincon(@(x)DiffRob_cost(x,Ts,Np,th,obs,n_obs,st_ref,st_0),x0,[],[],C,d,0,q,myoptions);
    u = xstar;
    ustar               =       [u(1:Np,1)';
                            u(Np+1:2*Np,1)'];
    u_mpc = [xstar(1,1);xstar(Np+1,1)];

    xi_sim      =   zeros(3,Np+1);
    xi_sim(:,1) =   st_0;

    % Compute Optimal Trajectory for visualization
    for ind=1:Np+1
        xidot               =     diff_drive(0,xi_sim(:,ind),ustar,th);
        xi_sim(:,ind+1)       =   xi_sim(:,ind)+Ts*xidot;
    end
    xx1(:,1:3,n_iter+1)= full(xi_sim)';   % store trajectory for visualization
    u_cl= [u_cl ; u_mpc(1,1) u_mpc(2,1)]; %store u_mpc for visualizaton 
    t(n_iter+1) = t0;

    % Init next Horizon window
    [t0, st_0, u0] = move_horizon(Ts, t0, st_0, u, Np,th); 

    xx(:,n_iter+2) = st_0; %store state after executed u_mpc for visualization
    % Set next step
    n_iter
    n_iter = n_iter + 1;    
end
mpc_time = toc(mpc_loop);
error = norm(st_0(1:2,1)-st_ref(1:2,1),2)
average_mpc_time = mpc_time/(n_iter+1)
Robot_traj (t,xx,xx1,u_cl,goal,Np,rob_diam,x_,y_,w_map,h_map,th) % a drawing function