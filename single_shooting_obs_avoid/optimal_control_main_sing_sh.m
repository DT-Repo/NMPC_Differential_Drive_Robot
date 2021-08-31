% Constrained Numerical Optimization for Estimation and Control Project
%
% Non linear MPC - Single Shooting for a differential-drive robot 
% with obstacle avoidance.

clear all
close all
clc

%% Model parameters
base           =       0.235;      %distance between two wheels [m]
r           =       0.036;      %radius robot's wheels [m]
rob_diam    =       0.3485;     %robot's size [m]
th          =       [r;base];

%% Envinroment parameters
n_obs   =       12;           %number of obstacles           
h_map   =       8;           %height of the map
w_map   =       10;          %width of the map

%% Generate map
%generate a map with random obstacles shaped as circles
load('x_test');
load('y_test');
load('xc_test');
load('yc_test');
load('rad_test');
%[x_,y_,xc,yc,rad]   =     generate_map(w_map,h_map,n_obs);
obs               =     [xc,yc,rad];
%% Define start and goal
start = [rob_diam/2;rob_diam/2;0];        
goal = [8;7;0];

%% FHOCP parameters - Single Shooting
Ts      =       0.5;                % seconds, input sampling period
Tend    =       40;                 % seconds, terminal time
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
while(norm((st_0(1:2,1)-st_ref(1:2,1)),2) > 1e-2 && n_iter < Tend / Ts)
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
error = norm(st_0-st_ref,2)
average_mpc_time = mpc_time/(n_iter+1)
Robot_traj (t,xx,xx1,u_cl,goal,Np,rob_diam,x_,y_,w_map,h_map,th) % a drawing function
% %% Casadi Solver's Solution
% addpath('.\casadi-windows-matlabR2016a-v3.4.5')
% import casadi.*
% 
% d           =       0.235;      %distance between two wheels [m]
% r           =       0.036;      %radius robot's wheels [m]
% rob_diam    =       0.3485;     %robot's size [m]
% th          =       [r;d];
% 
% 
% n_obs   =       12;           %number of obstacles           
% h_map   =       8;           %height of the map
% w_map   =       10;          %width of the map
% 
% 
% %generate a map with random obstacles shaped as circles
% % [x_,y_,xc,yc,rad]   =     generate_map(w_map,h_map,n_obs);
% % load('x_cas');
% % load('y_cas');
% % load('xc_cas');
% % load('yc_cas');
% % load('rad_cas');
% safety_dist     =       0.3485/2;
% obs               =     [xc,yc,rad];
% 
% start = [rob_diam/2;rob_diam/2;0];        
% 
% omega_max   =       5.5; %[rad/s]
% v_max       =       0.648; %[m/s]  
% v_min = -v_max;
% omega_min = -omega_max;
% 
% x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
% states = [x;y;theta]; n_states = length(states);
% 
% v = SX.sym('v'); omega = SX.sym('omega');
% controls = [v;omega]; n_controls = length(controls);
% model = [v*cos(theta);v*sin(theta);omega]; % system r.h.s
% 
% f = Function('f',{states,controls},{model}); % nonlinear mapping function f(x,u)
% U = SX.sym('U',n_controls,Np); % Decision variables (controls)
% P = SX.sym('P',n_states + n_states);
% 
% X = SX.sym('X',n_states,(Np+1));
% 
% 
% % compute solution symbolically
% X(:,1) = P(1:3); % initial state
% for k = 1:Np
%     st = X(:,k);  con = U(:,k);
%     f_value  = f(st,con);
%     st_next  = st + (Ts*f_value);
%     X(:,k+1) = st_next;
% end
% % this function to get the optimal trajectory knowing the optimal solution
% ff=Function('ff',{U,P},{X});
% 
% obj = 0; % Objective function
% g = [];  % constraints vector
% 
% Q = zeros(3,3); Q(1,1) = 1;Q(2,2) = 1;Q(3,3) = 1; % weighing matrices (states)
% R = zeros(2,2); R(1,1) = 1; R(2,2) = 1; % weighing matrices (controls)
% % compute objective
% for k=1:Np
%     st = X(:,k);  con = U(:,k);
%     obj = obj+(st-P(4:6))'*Q*(st-P(4:6)) + con'*R*con; % calculate obj
% end
% 
% % compute constraints
% for k = 1:Np+1   % box constraints due to the map margins
%     g = [g ; X(1,k)];   %state x
%     g = [g ; X(2,k)];   %state y
% end
% 
% for k = 1:Np+1   % box constraints due to the map margins
%     for i = 1:n_obs
%         g = [g ; -sqrt((X(1,k)-xc(i,1))^2+(X(2,k)-yc(i,1))^2) + (safety_dist + rad(i,1))];
%     end
% end
% % make the decision variables one column vector
% OPT_variables = reshape(U,2*Np,1);
% nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);
% 
% opts = struct;
% opts.ipopt.max_iter = 50;
% opts.ipopt.print_level =0;%0,3
% opts.print_time = 0;
% opts.ipopt.acceptable_tol =1e-8;
% opts.ipopt.acceptable_obj_change_tol = 1e-6;
% 
% solver = nlpsol('solver', 'ipopt', nlp_prob,opts);
% 
% 
% args = struct;
% 
% %define linear constraints for states
% args.lbg(1:(Np+1)) = 0; 
% args.ubg(1:(Np+1)) = (w_map-0.1742); 
% args.lbg(Np+2:2*(Np+1)) = 0; 
% args.ubg(Np+2:2*(Np+1)) = (h_map-0.1742); 
% 
% %Define range for non linear constraints cost function
% args.lbg(2*(Np+1)+1 : (n_obs+2)*(Np+1)) = -inf; % inequality constraints
% args.ubg(2*(Np+1)+1 : (n_obs+2)*(Np+1)) = 0; % inequality constraints
% 
% % input constraints
% args.lbx(1:2:2*Np-1,1) = v_min; 
% args.lbx(2:2:2*Np,1)   = omega_min;
% args.ubx(1:2:2*Np-1,1) = v_max; 
% args.ubx(2:2:2*Np,1)   = omega_max;
% 
% 
% t0 = 0;
% x0 = start;    % initial condition.
% 
% xx(:,1) = x0; % xx contains the history of states
% t(1) = t0;
% 
% u0 = zeros(Np,2);  % two control inputs 
% 
% 
% % Start MPC
% n_iter_cas = 0;
% xx1 = [];
% u_cl=[];
% 
% 
% mpc_loop_cas = tic;
% while(norm((x0-goal),2) > 1e-2 && n_iter_cas < Tend / Ts)
%     args.p   = [x0;goal]; % set the values of the parameters vector
%     args.x0 = reshape(u0',2*Np,1); % initial value of the optimization variables
%     
%     %tic
%     sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
%             'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
%     %toc
%     
%     u = reshape(full(sol.x)',2,Np)';
%     ff_value = ff(u',args.p); % compute OPTIMAL solution TRAJECTORY
%     xx1(:,1:3,n_iter_cas+1)= full(ff_value)';
%     
%     u_cl= [u_cl ; u(1,:)];
%     t(n_iter_cas+1) = t0;
%     [t0, x0, u0] = shift(Ts, t0, x0, u,f); % get the initialization of the next optimization step
%     
%     xx(:,n_iter_cas+2) = x0;  
%     n_iter_cas
%     n_iter_cas = n_iter_cas + 1;
% end;
% mpc_time_cas = toc(mpc_loop_cas)
% error = norm(x0-goal,2)
% average_mpc_time_cas = mpc_time_cas/(n_iter_cas+1)
% % Draw_MPC_point_stabilization_v1 (t,xx,xx1,u_cl,goal,Np,rob_diam) % a drawing function
% Robot_traj (t,xx,xx1,u_cl,goal,Np,rob_diam,x_,y_,w_map,h_map,th) % a drawing function
% disp('------------------------------------------------------------------')
% disp('fmincon :')
% n_iter
% 
% average_mpc_time
% disp('casadi :')
% n_iter_cas
% 
% average_mpc_time_cas

