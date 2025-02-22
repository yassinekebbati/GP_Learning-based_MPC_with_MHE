%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 10/05/2022
% Control NMMPC-NMHE-Racing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% point stabilization + Single shooting
clear all
close all
clc


% CasADi v3.5.5
addpath('casadi-windows-matlabR2016a-v3.5.5')
import casadi.*

%% Initialize vehicle model parameters 
lf= 0.125;
lr= 0.125;
C1 = lr/(lr+lf);
C2 = 1/(lr+lf);
Cr0 = 0.6;
Cr2 = 0.1;
Cm1 = 12.0;
Cm2 = 2.17;


% %% Initialize parameters for Normal Oval track=================================================================================================================
% % 
% % MPC Q weights
% Qy = 0.015;
% Qx = 0.015;
% Qphi = 0;
% Qv = 0.00015;
% 
% % MPC R weights
% Qdelta = 0.003;
% Qa = 0.0025;
% % 
% % MPC R2 weights for rate of control
% Qddelta = 0.005;
% Qda = 0.00053;
% 
% % MPC paramaetrs
% T = 0.033;                  % sampling time [s]
% sim_tim = 2*6;              % Maximum simulation time
% N_samples = round(sim_tim/T);
% N = 14;                     % prediction horizon
% L = 5                       %look ahead
% 
% % load trajectory
% load trackNO
% load RefNO
% Xp = xRef; 
% Yp = yRef;
% xs = [Xp ; Yp ; zeros(1,length(Yp)); vxRef(1:length(Yp))];% vyRef; wRef]; % Reference posture.
% load O_nocorrection %% load learned GP corrections for Oval track
% %%=================================================================================================================



%% Initialize parameters for L track =================================================================================================================

% % % % MPC Q weights
Qy = 0.015;
Qx = 0.015;
Qphi = 0;
Qv = 0.00015;

% % % % MPC R weights
Qdelta = 0.003;
Qa = 0.0025;

% % % % MPC R2 weights for rate of control
Qddelta = 0.005;
Qda = 0.00053;

% % % % MPC paramaetrs
T = 0.033;              % sampling time [s]
sim_tim = 13.86;        % Maximum simulation time
N_samples = round(sim_tim/T);
N = 9;                  % prediction horizon 9
L = 0;                  %look ahead

% % % load trajectory
load trackL
load RefL
load RacelineL
xs = [Xp ; Yp ; zeros(1,length(Yp)); vxRef(1:length(Yp))]; % Reference posture.
% load L_correction %% load learned GP corrections for L track

%% =================================================================================================================



% 
% %% Initialize parameters for Oval track =================================================================================================================
% 
% % MPC Q weights
% Qy = 0.015;
% Qx = 0.015;
% Qphi = 0;
% Qv = 0.00015;
% 
% % MPC R weights
% Qdelta = 0.003;
% Qa = 0.0025;
% 
% 
% % MPC R2 weights for rate of control
% Qddelta = 0.005;
% Qda = 0.00053;
% 
% % MPC paramaetrs
% T = 0.033;                  % sampling time [s]
% sim_tim = 17;               % Maximum simulation time for 2 laps
% N_samples = round(sim_tim/T);
% N = 9;                      % prediction horizon
% L = 0;                      %look ahead 
% 
% % load trajectory
% load trackO;
% load RefO;
% load VxO;
% load RacelineO;
% xs = [Xp ; Yp ; zeros(1,length(Yp)); vxRef(1:length(Yp))];% vyRef; wRef]; % Reference posture.
% load O_correction %% load learned GP corrections for Oval track
% %%=================================================================================================================




%% load neural network for NN correction (both Oval and L tracks)
load net_ex;
load net_ey;
load net_epsi;


% Initialize vehicle states for simulation
vehicle_states  = [Xp(1); Yp(1); 0; 0; 0; 0];

% Measurments noise covariance 
con_cov = diag([0.02 deg2rad(2)]).^2;
meas_cov = diag([0.05 0.05 deg2rad(2) 0.02]).^2;
V = inv(sqrt(meas_cov));                            % weighing matrices (output)  y_tilde - y
W = inv(sqrt(con_cov));                             % weighing matrices (input)   u_tilde - u

% Set control bounds
delta_max = pi/6; delta_min = -delta_max;
a_max = 1; a_min = -1;

% Start casadi code
x = SX.sym('x'); y = SX.sym('y'); phi = SX.sym('phi'); v = SX.sym('v'); %phi = SX.sym('phi'); w = SX.sym('w'); 
states = [x;y;phi;v]; n_states = length(states);


delta = SX.sym('delta'); a = SX.sym('a');
controls = [delta;a]; n_controls = length(controls);

% Right hand side
rhs = [v*cos(phi+C1*delta); v*sin(phi+C1*delta); v*delta*C2; (Cm1-Cm2*v)*a-Cr2*v^2-Cr0-(v*delta)^2*C2*C1^2];


f = Function('f',{states,controls},{rhs});      % nonlinear mapping function f(x,u)
U = SX.sym('U',n_controls,N);                   % Decision variables (controls)
P = SX.sym('P',n_states + n_states);

% parameters (which include the initial and the reference state of the robot)

X = SX.sym('X',n_states,(N+1));

% A Matrix that represents the states over the optimization problem.

obj = 0; % Objective function
g = [];  % constraints vector

Q = zeros(2,2); Q(1,1) = Qx;Q(2,2) = Qy; Q(3,3) = Qphi; Q(4,4) = Qv; %Q(5,5) = Qvy; Q(6,6) = Qw; % weighing matrices (states)
R = zeros(2,2); R(1,1) = Qdelta; R(2,2) = Qa;               % weighing matrices (controls)
R1 = zeros(2,2); R1(1,1) = Qddelta; R1(2,2) = Qda;          % weighing matrices (control rate)

%% Multiple shooting (using U and X as decision variable)

st = X(:,1);                %initiale state
g = [g; st - P(1:4)];       %inital condition constraints
for k = 1:N
    st = X(:,k);  con = U(:,k); 
    obj = obj+(st(1:4)-P(5:8))'*Q*(st(1:4)-P(5:8)) + con'*R*con; % calculate obj
%     obj = obj+(st(4)-4)'*Qv*(st(4)-4) + con'*R*con;
    st_next  = X(:,k+1);
    f_value  = f(st,con);
    st_next_euler  = st+ (T*f_value);
    g = [g; st_next - st_next_euler];   %compute constraints
end

%%%%%%%%%%%%%%%%%%%%% terminal cost
obj = obj+(st(1:4)-P(5:8))'*Q*(st(1:4)-P(5:8));



% make the decision variables one column vector
OPT_variables = [reshape(X,n_states*(N+1),1);reshape(U,2*N,1)];
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

% MPC solver
solver = nlpsol('solver', 'ipopt', nlp_prob,opts);


% MPC state bounds
args = struct;

% Euality constraints 
args.lbg(1:n_states*(N+1)) = 0;%0.001;  % lower bound of the states x and y
args.ubg(1:n_states*(N+1)) = 0;%25;   % upper bound of the states x and y 


% %state constraints
args.lbx(1:n_states:n_states*(N+1),1) = -inf; 
args.ubx(1:n_states:n_states*(N+1),1) = inf; 
args.lbx(2:n_states:n_states*(N+1),1) = -inf;   %max(RightSide(2,i),LeftSide(2,i)); 
args.ubx(2:n_states:n_states*(N+1),1) = inf;    %max(RightSide(2,i),LeftSide(2,i));
args.lbx(3:n_states:n_states*(N+1),1) = -inf; 
args.ubx(3:n_states:n_states*(N+1),1) = inf;
args.lbx(4:n_states:n_states*(N+1),1) = 0.1;    %-inf; 
args.ubx(4:n_states:n_states*(N+1),1) = 5;      %inf;


% input constraints
args.lbx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = delta_min; 
args.ubx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = delta_max; 
args.lbx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1)   = a_min;
args.ubx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1)   = a_max;

% ALL OF THE ABOVE IS MPC PROBLEM SETTING UP
%%

% MHE problem formulation starts here

Tm = 0.033; % Discretization time
N_MHE = 6; % size(y_measurements,1)-1; % Prediction horizon


% rhs2 = [v*cos(phi+C1*delta); v*sin(phi+C1*delta); v*delta*C2; (Cm1-Cm2*v)*a-Cr2*v^2-Cr0-(v*delta)^2*C2*C1^2]; % system r.h.s


% f2 = Function('f',{states,controls},{rhs}); % nonlinear mapping

% measurement_rhs = [v*cos(phi+C1*delta)+ sqrt(meas_cov(1,1))*randn(1); v*sin(phi+C1*delta)+sqrt(meas_cov(2,2))*randn(1); v*delta*C2+sqrt(meas_cov(3,3))*randn(1); (Cm1-Cm2*v)*a-Cr2*v^2-Cr0-(v*delta)^2*C2*C1^2 + sqrt(meas_cov(4,4))*randn(1)];
% measurement_rhs = [x - sqrt(meas_cov(1,1))*randn(1); y - sqrt(meas_cov(2,2))*randn(1); phi - sqrt(meas_cov(3,3))*randn(1); v - sqrt(meas_cov(4,4))*randn(1)];
measurement_rhs = [x ; y ; phi; v];

h = Function('h',{states,controls},{measurement_rhs}); % MEASUREMENT MODEL

% Decision variables
U2 = SX.sym('U',n_controls,N_MHE);    %(controls)
X2 = SX.sym('X',n_states,(N_MHE+1));  %(states) [remember multiple shooting]

P2 = SX.sym('P', n_states+n_controls , (N_MHE+1));
% parameters (include state measurements as well as controls measurements)

obj2 = 0; % Objective function
g2 = [];  % constraints vector

for k = 1:N_MHE+1
    st2 = X2(:,k);
    if k<N_MHE+1
        con2 = U2(:,k);
    end
    h_x = h(st2,con2);
    y_tilde = P2(1:4,k);
    obj2 = obj2+ (y_tilde-h_x)' *V* (y_tilde-h_x); % calculate obj
end


%%%%%%%%%%%%%%%%%%%%% terminal cost 
obj2 = obj2+ (y_tilde-h_x)' *V* (y_tilde-h_x);

for k = 1:N_MHE
%     con = U(:,k);
    u_tilde = P2(5:6,N_MHE);
    obj2 = obj2+ (u_tilde-con2)' *W* (u_tilde-con2); % calculate obj
end

% multiple shooting constraints
for k = 1:N_MHE
    st2 = X2(:,k);  con2 = U2(:,k);
    st_next2 = X2(:,k+1);
    f_value2 = f(st2,con2);
    st_next_euler2 = st2+ (Tm*f_value2);
    g2 = [g2;st_next2-st_next_euler2]; % compute constraints
end


% make the decision variable one column  vector
OPT_variables2 = [reshape(X2,n_states*(N_MHE+1),1);reshape(U2,n_controls*N_MHE,1)];

nlp_mhe = struct('f', obj2, 'x', OPT_variables2, 'g', g2, 'p', P2);

opts2 = struct;
opts2.ipopt.max_iter = 2000;
opts2.ipopt.print_level =0;%0,3
opts2.print_time = 0;
opts2.ipopt.acceptable_tol =1e-8;
opts2.ipopt.acceptable_obj_change_tol = 1e-6;

% MHE solver
solver2 = nlpsol('solver', 'ipopt', nlp_mhe,opts2);

% MHE bounds
args2 = struct;

args2.lbg(1:n_states*(N_MHE)) = 0;
args2.ubg(1:n_states*(N_MHE)) = 0;

args2.lbx(1:n_states:n_states*(N_MHE+1),1) = -inf; %state x lower bound
args2.ubx(1:n_states:n_states*(N_MHE+1),1) = inf; %state x upper bound
args2.lbx(2:n_states:n_states*(N_MHE+1),1) = -inf; %state y lower bound
args2.ubx(2:n_states:n_states*(N_MHE+1),1) = inf; %state y upper bound
args2.lbx(3:n_states:n_states*(N_MHE+1),1) = -inf; %state theta lower bound
args2.ubx(3:n_states:n_states*(N_MHE+1),1) = inf; %state theta upper bound
args2.lbx(4:n_states:n_states*(N_MHE+1),1) = -inf; %state theta lower bound
args2.ubx(4:n_states:n_states*(N_MHE+1),1) = inf; %state theta upper bound

args2.lbx(n_states*(N_MHE+1)+1:n_controls:n_states*(N_MHE+1)+2*N_MHE,1) = delta_min; %v lower bound
args2.ubx(n_states*(N_MHE+1)+1:n_controls:n_states*(N_MHE+1)+2*N_MHE,1) = delta_max; %v upper bound
args2.lbx(n_states*(N_MHE+1)+2:n_controls:n_states*(N_MHE+1)+2*N_MHE,1) = a_min; %omega lower bound
args2.ubx(n_states*(N_MHE+1)+2:n_controls:n_states*(N_MHE+1)+2*N_MHE,1) = a_max; %omega upper bound
%----------------------------------------------
% ALL OF THE ABOVE IS MHE PROBLEM SET UP



%% Parameter initialization for simulation 
t0 = 0;
x0 = [xRef(1) ; yRef(1) ; yawRef(1); vxRef(1)];    % initial condition.


xx(:,1) = [xs(1:3,1);0]; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,n_controls);  % two control inputs 
X0 = repmat(x0,1,N+1);


i = 0;
xx1 = [];

u_cl=[];
y_measurements = [];
u_measurements = [];

X_estimate = []; % X_estimate contains the MHE estimate of the states
U_estimate = []; % U_estimate contains the MHE estimate of the controls

U02 = zeros(N_MHE,n_controls);   % two control inputs for each robot
X02 = zeros(N_MHE+1,n_states); % initialization of the states decision variables

t_lap = [];
it = [];

%% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------

main_loop = tic;

for i = 1:sim_tim/T
    
%     
%     args.lbx(1:n_states:n_states*(N+1),1) = -inf;%xx(1,i)-0.5; 
%     args.ubx(1:n_states:n_states*(N+1),1) = xx(1,i)+0.5;; 
%     args.lbx(2:n_states:n_states*(N+1),1) = xx(2,i)-0.5; 
%     args.ubx(2:n_states:n_states*(N+1),1) = xx(2,i)+0.5;
%     
    
    if xx(1,i) > xRef(1) && xx(2,i) < 0.5
       
        t_lap = [t_lap i];
%         disp('lap time : ', t_lap(i))
        
    end
    
%% Neural Network correction======================================================================================================= 
    xx0(:,i) = x0;
    args.p   = [xx0(:,i);xs(:,i+L)]; % set the values of the parameters vector

    if i>1 
        ex = pred([(xx(:,i-1) - xx0(:,i-1));u_cl(i-1,:)';xx(:,i)] ,xmeanx', xstdevx', ymeanx,ystdevx', modx);
        ey = pred([(xx(:,i-1) - xx0(:,i-1));u_cl(i-1,:)';xx(:,i)] ,xmeany', xstdevy', ymeany,ystdevy', mody);
        epsi = pred([(xx(:,i-1) - xx0(:,i-1));u_cl(i-1,:)';xx(:,i)] ,xmeanpsi', xstdevpsi', ymeanpsi,ystdevpsi', modpsi);
        xx0(:,i) = x0+[ex;ey;epsi;0];
    else
        xx0(:,i) = x0;
    end
%%==========================================================================================================================


   
% %% Gaussian process (GP) correction=============================================================================================================
%     %%correction
%     xx0(:,i) = x0+[e_x(i);e_y(i);e_psi(i);e_vx(i)];
%     args.p   = [xx0(:,i);xs(:,i+L)]; % set the values of the parameters vector    with gp correction added
% 
% %% =========================================================================================================================
    
    args.x0 = [reshape(X0,n_states*(N+1),1);reshape(u0',n_controls*N,1)]; % initial value of the optimization variables
    
    tic
    
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
      
    ET_MPC(i) = toc;  %mpc computation time
    
    u = reshape(full(sol.x(n_states*(N+1)+1:end))',n_controls,N)';
        
%     ff_value = ff(u',args.p); % compute OPTIMAL solution TRAJECTORY
%     xx1(:,1:4,i+1)= full(ff_value)';
        
    xx1(:,1:n_states,i+1)= reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)';
    
    % Get solution trajectory
    u_cl= [u_cl ; u(1,:)];
    

    
    
    Ts = T*i:T/30:T*i+T;
    [x0, u0, vehicle_states] = shift(Ts, xx(:,i), u, vehicle_states); % get the initialization of the next optimization step using full car model (Sumilate vihecle) 

    
    xx(:,i+1) = x0; 
    
    %synthesize noisy measurments
%     y_measure = [ x0(1,1); x0(2,1); x0(3,1) ; x0(4,1)];  %no noise included 
%     u_measure = [u(1,1) ; u(1,2)];                       %no noise included 
    
    % noise included here
    y_measure = [ x0(1,1) + sqrt(meas_cov(1,1))*randn(1) ; x0(2,1) + sqrt(meas_cov(2,2))*randn(1); x0(3,1) + sqrt(meas_cov(3,3))*randn(1); x0(4,1) + sqrt(meas_cov(4,4))*randn(1)];
    u_measure = [u(1,1) + sqrt(con_cov(1,1))*randn(1) ; u(1,2) + sqrt(con_cov(2,2))*randn(1)];
    y_measurements = [ y_measurements y_measure];
    u_measurements = [u_measurements u_measure];
    

    X0 = reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)';  %get solution trajecotry
    
    % shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
    
    
    %% MHE estimation starts here
    %----------------------------
    
    if i >= N_MHE+1 %check moving horizon window

    % Get the measurements window and put it as parameters in p
    args2.p2   = [y_measurements(:,i-N_MHE:i);u_measurements(:,i-N_MHE:i)];

    % initial value of the optimization variables
    args2.x0  = [reshape(X02,n_states*(N_MHE+1),1);reshape(U02',n_controls*N_MHE,1)];
    
    tic
    sol2 = solver2('x0', args2.x0, 'lbx', args2.lbx, 'ubx', args2.ubx,...
        'lbg', args2.lbg, 'ubg', args2.ubg,'p',args2.p2);
    ET_MHE(i) = toc; %mhe time
    
    U_sol = reshape(full(sol2.x(n_states*(N_MHE+1)+1:end))',n_controls,N_MHE)'; % get controls only from the solution
    X_sol = reshape(full(sol2.x(1:n_states*(N_MHE+1)))',n_states,N_MHE+1)'; % get solution TRAJECTORY
    X_estimate = [X_estimate;X_sol(N_MHE+1,:)];
    U_estimate = [U_estimate;U_sol(N_MHE,:)];
    
    % Shift trajectory to initialize the next step
    X02 = [X_sol(2:end,:);X_sol(end,:)];
    U02 = [U_sol(2:end,:);U_sol(end,:)];
    x0 = X_estimate(i-N_MHE,:)';
   
    end
    
    %%
%     i
    it = [it i]; %save iteration
    i = i + 1;  %increment
    
    figure(1)
    subplot(2,1,1); plot(u_cl(:,2),'-.k','linewidth',1.5), ylabel('\alpha [m/s^2]'); xlabel('iteration'); grid on;
    subplot(2,1,2); plot(u_cl(:,1),'-.k','linewidth',1.5), ylabel('\delta [rad]'); xlabel('iteration'); grid on;  
    
    figure(2)
    
    plot(RightSide(1,:),RightSide(2,:),'k', LeftSide(1,:),LeftSide(2,:),'k'); hold on; plot(xx(1,1:i),xx(2,1:i),'ok');ylabel('Y_p [m]');xlabel('X_p [m]'); grid on ;plot(xs(1,1:i),xs(2,1:i),'r')     

    
end;

%% Compute time and plot results
%-------------------------------

average_mpc_time = sum(ET_MPC)/i

average_mhe_time = sum(ET_MHE)/i

main_loop_time = toc(main_loop)

msex = norm((xx0(1,1:N_samples-1)-xs(1,1:N_samples-1))/i,2)

msey = norm((xx0(2,1:N_samples-1)-xs(2,1:N_samples-1))/i,2)

max_velocity = max(xx0(4,:)) 

j= i-1;



%% Plot racing results =================================================================================================================

figure (4); %% ploting the controls ground truth, measurments.
subplot(2,1,1); plot(u_cl(:,1),'linewidth',1.5), ylabel('\delta [rad]'), hold on, plot(u_measurements(1,:),'-.r','linewidth',1), grid on; 
subplot(2,1,2); plot(u_cl(:,2),'linewidth',1.5), ylabel('\alpha [m/s^2]'), hold on, plot(u_measurements(2,:),'-.r','linewidth',1), xlabel('iteration'), grid on;


figure(5);  %% ploting the tracking error
subplot(2,1,1); plot(xs(1,1:j-N_MHE)-X_estimate(1:j-N_MHE,1)','k','linewidth',1.5), ylabel('Xp tracking error [m]'),xlabel('iteration'), grid on;
subplot(2,1,2); plot(xs(2,1:j-N_MHE)-X_estimate(1:j-N_MHE,2)','k','linewidth',1.5), ylabel('Yp tracking error [m]'),xlabel('iteration'), grid on;

figure (6);  %%ploting the states ground truth, measurments, estimations.
subplot(4,1,1); plot(it(1:j),xx(1,1:j),'linewidth',1.5), ylabel('X_p [m]'), hold on, plot(it(1:j), y_measurements(1,1:j),'r','linewidth',0.5), plot(it(N_MHE:j-1), X_estimate(1:j-N_MHE,1),'g','linewidth',1); grid on
subplot(4,1,2); plot(it(1:j),xx(2,1:j),'linewidth',1.5), ylabel('Y_p [m]'), hold on, plot(it(1:j), y_measurements(2,1:j),'r','linewidth',0.5), plot(it(N_MHE:j-1), X_estimate(1:j-N_MHE,2),'g','linewidth',1); grid on
subplot(4,1,3); plot(it(1:j),xx(3,1:j),'linewidth',1.5), ylabel('\phi [rad]'), grid on , hold on, plot(it(1:j), y_measurements(3,1:j),'r','linewidth',0.5), plot(it(N_MHE:j-1), X_estimate(1:j-N_MHE,3),'g','linewidth',1); grid on
subplot(4,1,4); plot(it(1:j),xx(4,1:j),'linewidth',1.5), ylabel('v_x [m/s]'), grid on , hold on, plot(it(1:j), y_measurements(4,1:j),'r','linewidth',0.5), plot(it(N_MHE:j-1), X_estimate(1:j-N_MHE,4),'g','linewidth',1); 
%%=================================================================================================================

