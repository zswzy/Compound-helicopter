% 2.28 Rotorcraft trim
clear all
clc
h = 100; % flight altitude
[~,~,~,rho] = atmosisa(h);
table_trim_states = readtable('trim_result_no_redundant.csv');
%% build object
run init_build.m

%% hover/foward trim X,Y,Z,L,M,N=0,[theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
Rotorcraft.DoubleRotorHelicopter.U         = 84;
Rotorcraft.DoubleRotorHelicopter.V         = 0;
Rotorcraft.DoubleRotorHelicopter.W         = 0;
Rotorcraft.DoubleRotorHelicopter.U_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.V_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.W_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.p         = 0;
Rotorcraft.DoubleRotorHelicopter.q         = 0;
Rotorcraft.DoubleRotorHelicopter.r         = 0;
Rotorcraft.DoubleRotorHelicopter.p_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.q_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.r_dot     = 0;

nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U+1),2:9};
% nearest_initial_no_redundant = [0.01,0,0,0,0,0,10,10];
% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
cell_InitialStates      = {nearest_initial_no_redundant [0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                            [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                            [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                            [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
[x_trim,~,~,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                @Aerodynamics_trim_full_8var, ... 
                                cell_InitialStates, ...
                                options, ...
                                2, ...                  % LowerRotor.inteference
                                2, ...                  % UpperRotor.inteference
                                deg2rad(20), ...        % Prop.theta_0
                                0, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                deg2rad(0), ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                deg2rad(0), ...         % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                deg2rad(0), ...         % theta_1c_diff
                                deg2rad(0));            % theta_1s_diff
info_dynamics(Rotorcraft)

%% Search the required power under different propeller pitch
Rotorcraft.DoubleRotorHelicopter.U         = 100; 
Rotorcraft.DoubleRotorHelicopter.V         = 0;
Rotorcraft.DoubleRotorHelicopter.W         = 0;
Rotorcraft.DoubleRotorHelicopter.U_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.V_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.W_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.p         = 0;
Rotorcraft.DoubleRotorHelicopter.q         = 0;
Rotorcraft.DoubleRotorHelicopter.r         = 0;
Rotorcraft.DoubleRotorHelicopter.p_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.q_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.r_dot     = 0;

array_Prop_theta_0  = linspace(deg2rad(0),deg2rad(40),40);
array_power         = zeros(size(array_Prop_theta_0));

nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:9};
x_trim_last = nearest_initial_no_redundant;
for j = 1:40
    ele_Prop_theta_0 = array_Prop_theta_0(j);
    % x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
    options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
    cell_InitialStates      = {x_trim_last [0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                                [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                                [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                                [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
    [x_trim,~,~,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                    @Aerodynamics_trim_full_8var, ... 
                                    cell_InitialStates, ...
                                    options, ...
                                    2, ...                  % LowerRotor.inteference
                                    2, ...                  % UpperRotor.inteference
                                    ele_Prop_theta_0, ...   % Prop.theta_0
                                    1, ...                  % Prop.isEnable
                                    1, ...                  % Fus.isEnable
                                    deg2rad(0), ...         % HorStab.delta_e
                                    1, ...                  % HorStab.isEnable
                                    deg2rad(0), ...         % VerStab.delta_r
                                    1, ...                  % VerStab.isEnable
                                    deg2rad(0), ...         % theta_1c_diff
                                    deg2rad(0));            % theta_1s_diff
    x_trim_last = x_trim;
    array_power(j) = power_total;
end
plot(rad2deg(array_Prop_theta_0),array_power/1000,'linewidth',1.5)
xlabel('Prop \theta_0 (deg)');ylabel('Total power (kW)');title(['Total power, U = ',num2str(Rotorcraft.DoubleRotorHelicopter.U)]);grid on;

%% Search the best propeller pitch under a specific velocity 2.20 optim algo fmincon
tic;
Rotorcraft.DoubleRotorHelicopter.U         = 30; 
Rotorcraft.DoubleRotorHelicopter.V         = 0;
Rotorcraft.DoubleRotorHelicopter.W         = 0;
Rotorcraft.DoubleRotorHelicopter.U_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.V_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.W_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.p         = 0;
Rotorcraft.DoubleRotorHelicopter.q         = 0;
Rotorcraft.DoubleRotorHelicopter.r         = 0;
Rotorcraft.DoubleRotorHelicopter.p_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.q_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.r_dot     = 0;

% initial point for trimming
nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:9};

% if propeller is enabled
% choose several initial point for global minimum [Prop_theta_0, Prop_isEnable]
cell_redundant_var_initial  = { deg2rad(5), ...
                                deg2rad(17), ...
                                deg2rad(30)}; % cell of initial point
array_redundant_var_best        = [ones(3,2);0 0];
array_power_best                = zeros(4,1);

% search the (local) best solutions 
for j = 1:3
    problem = struct;
    problem.options = optimoptions('fmincon','Display','iter','PlotFcns',[],'algorithm','sqp');
    problem.solver = 'fmincon';
    problem.objective = @(x) trim_power(Rotorcraft,nearest_initial_no_redundant,x,1);
    problem.x0 = cell_redundant_var_initial{j};
    problem.lb = 0;
    problem.ub = deg2rad(40);
    
    [array_redundant_var_best(j,1),array_power_best(j)] = fmincon(problem);
end

% if propeller is NOT enabled
array_power_best(4) = trim_power(Rotorcraft,nearest_initial_no_redundant,0,0);

% Search the global minimum among the local minimum results
power_best              = min(array_power_best);
redundant_var_best      = array_redundant_var_best(find(array_power_best == power_best),:);
redundant_var_best      = redundant_var_best(1,:);
toc

%% Search the best propeller pitch under a specific velocity 2.28 Genetic algo ga
tic;
Rotorcraft.DoubleRotorHelicopter.U         = 84.5; 
Rotorcraft.DoubleRotorHelicopter.V         = 0;
Rotorcraft.DoubleRotorHelicopter.W         = 0;
Rotorcraft.DoubleRotorHelicopter.U_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.V_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.W_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.p         = 0;
Rotorcraft.DoubleRotorHelicopter.q         = 0;
Rotorcraft.DoubleRotorHelicopter.r         = 0;
Rotorcraft.DoubleRotorHelicopter.p_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.q_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.r_dot     = 0;

% initial point for trimming
nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:9};

array_redundant_var_best        = [0 1;0 0];
array_power_best                = zeros(2,1);

% if propeller is enabled
problem             = struct;
problem.options     = optimoptions('ga','Display','iter','FunctionTolerance',50);
problem.fitnessfcn  = @(x) trim_power(Rotorcraft,nearest_initial_no_redundant,x,1);
problem.lb          = 0;
problem.ub          = deg2rad(40);
problem.nvars       = 1;

[array_redundant_var_best(1,1) ,array_power_best(1)] = ga(problem);
% if propeller is NOT enabled
array_power_best(2) = trim_power(Rotorcraft,nearest_initial_no_redundant,0,0);

% Search the global minimum among the local minimum results
power_best              = min(array_power_best);
redundant_var_best      = array_redundant_var_best(find(array_power_best == power_best),:);
toc

%% Search the best propeller pitch under a specific velocity 2.28 differential evolution de 
tic;
Rotorcraft.DoubleRotorHelicopter.U         = 80; 
Rotorcraft.DoubleRotorHelicopter.V         = 0;
Rotorcraft.DoubleRotorHelicopter.W         = 0;
Rotorcraft.DoubleRotorHelicopter.U_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.V_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.W_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.p         = 0;
Rotorcraft.DoubleRotorHelicopter.q         = 0;
Rotorcraft.DoubleRotorHelicopter.r         = 0;
Rotorcraft.DoubleRotorHelicopter.p_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.q_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.r_dot     = 0;

% initial point for trimming
nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:9};

array_redundant_var_best        = [0 1;0 0];
array_power_best                = [0;0];

% if propeller is enabled
options.size_population     = 35;
options.max_generation      = 50;
options.scale_parameter     = 0.3;
options.crossover_proba     = 0.7;

problem.objective = @(x) trim_power(Rotorcraft,nearest_initial_no_redundant,x,1);
problem.lb = 0;
problem.ub = deg2rad(40);
problem.dimension = 1;

[array_redundant_var_best(1,1) ,array_power_best(1)] = de(problem,options);
% if propeller is NOT enabled
array_power_best(2) = trim_power(Rotorcraft,nearest_initial_no_redundant,0,0);

% Search the global minimum among the local minimum results
power_best              = min(array_power_best);
redundant_var_best      = array_redundant_var_best(find(array_power_best == power_best),:);
toc

%% Search the best redundant variables under a specific velocity 2.20 fmincon
tic
Rotorcraft.DoubleRotorHelicopter.U         = 80; 
Rotorcraft.DoubleRotorHelicopter.V         = 0;
Rotorcraft.DoubleRotorHelicopter.W         = 0;
Rotorcraft.DoubleRotorHelicopter.U_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.V_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.W_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.p         = 0;
Rotorcraft.DoubleRotorHelicopter.q         = 0;
Rotorcraft.DoubleRotorHelicopter.r         = 0;
Rotorcraft.DoubleRotorHelicopter.p_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.q_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.r_dot     = 0;

% initial point for trimming
nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:9};

% if propeller is enabled
% choose several initial point for global minimum [Prop_theta_0, Prop_isEnable=1, delta_e, delta_r,theta_1c_diff,theta_1s_diff]
cell_redundant_var_initial  = {[deg2rad(1),0,0,0,0], ...
                                [deg2rad(17),0,0,0,0], ...
                                [deg2rad(35),0,0,0,0]}; % cell of initial points
array_redundant_var_best        = [ones(3,6);0 0 0 0 0 0];
array_power_best                = zeros(6,1);

% search the (local) best solutions
for j = 1:3
    problem = struct;
    problem.options = optimoptions('fmincon','Display','iter','PlotFcns',[],'algorithm','sqp');
    problem.solver = 'fmincon';
    problem.objective = @(x) trim_power(Rotorcraft,nearest_initial_no_redundant,x(1),1,x(2),x(3),x(4),x(5));
    problem.x0 = cell_redundant_var_initial{j};
    problem.lb = [0,deg2rad(-25),deg2rad(-30),deg2rad(-1),deg2rad(-1)];
    problem.ub = [deg2rad(40),deg2rad(25),deg2rad(30),deg2rad(1),deg2rad(4.5)];
    
    [x, array_power_best(j)] = fmincon(problem);
    array_redundant_var_best(j,1) = x(1);
    array_redundant_var_best(j,3:6) = x(2:5);
end

% if propeller is NOT enabled
problem = struct;
problem.options = optimoptions('fmincon','Display','iter','PlotFcns',[],'algorithm','sqp');
problem.solver = 'fmincon';
problem.objective = @(x) trim_power(Rotorcraft,nearest_initial_no_redundant,0,0,x(1),x(2),x(3),x(4));
problem.x0 = [0,0,0,0];
problem.lb = [deg2rad(-25),deg2rad(-30),deg2rad(-1),deg2rad(-1)];
problem.ub = [deg2rad(25),deg2rad(30),deg2rad(1),deg2rad(4.5)];
[array_redundant_var_best(4,3:6),array_power_best(4)] = fmincon(problem);

% Search the global minimum among the local minimum results
power_best              = min(array_power_best);
redundant_var_best      = array_redundant_var_best(find(array_power_best == power_best),:);
redundant_var_best      = redundant_var_best(1,:);
toc


% calculte the correspond trimmed variables
% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
cell_InitialStates      = {[0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                            [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                            [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                            [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
[x,~,exitflag,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                @Aerodynamics_trim_full_8var, ... 
                                cell_InitialStates, ...
                                options, ...
                                2, ...                              % LowerRotor.inteference
                                2, ...                              % UpperRotor.inteference
                                redundant_var_best(1), ...          % Prop.theta_0
                                redundant_var_best(2), ...          % Prop.isEnable
                                1, ...                              % Fus.isEnable
                                redundant_var_best(3), ...          % HorStab.delta_e
                                1, ...                              % HorStab.isEnable
                                redundant_var_best(4), ...          % VerStab.delta_r
                                1, ...                              % VerStab.isEnable
                                redundant_var_best(5), ...          % theta_1c_diff
                                redundant_var_best(6));             % theta_1s_diff
                            
%% Search the best redundant variables under a specific velocity 2.28 genetic algo ga
tic
Rotorcraft.DoubleRotorHelicopter.U         = 80; 
Rotorcraft.DoubleRotorHelicopter.V         = 0;
Rotorcraft.DoubleRotorHelicopter.W         = 0;
Rotorcraft.DoubleRotorHelicopter.U_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.V_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.W_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.p         = 0;
Rotorcraft.DoubleRotorHelicopter.q         = 0;
Rotorcraft.DoubleRotorHelicopter.r         = 0;
Rotorcraft.DoubleRotorHelicopter.p_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.q_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.r_dot     = 0;

% initial point for trimming
nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:9};

% �ユ��灏炬��
array_redundant_var_best        = [ones(1,6);0 0 0 0 0 0];
array_power_best                = zeros(2,1);

% if propeller is enabled
problem             = struct;
problem.options     = optimoptions('ga','Display','iter','FunctionTolerance',100);
problem.fitnessfcn  = @(x) trim_power(Rotorcraft,nearest_initial_no_redundant,x(1),1,x(2),x(3),x(4),x(5));
problem.lb          = [0,deg2rad(-25),deg2rad(-30),deg2rad(-1),deg2rad(-1)];
problem.ub          = [deg2rad(40),deg2rad(25),deg2rad(30),deg2rad(1),deg2rad(4.5)];
problem.nvars       = 5;

[x,array_power_best(1)] = ga(problem);
array_redundant_var_best(1,1) = x(1);
array_redundant_var_best(1,3:6) = x(2:5);


% if propeller is NOT enabled
problem             = struct;
problem.options     = optimoptions('ga','Display','iter','FunctionTolerance',100);
problem.fitnessfcn  = @(x) trim_power(Rotorcraft,nearest_initial_no_redundant,0,0,x(1),x(2),x(3),x(4));
problem.lb          = [deg2rad(-25),deg2rad(-30),deg2rad(-1),deg2rad(-1)];
problem.ub          = [deg2rad(25),deg2rad(30),deg2rad(1),deg2rad(4.5)];
problem.nvars       = 4;

[array_redundant_var_best(2,3:6),array_power_best(2)] = ga(problem);

% Search the global minimum among the local minimum results
power_best              = min(array_power_best);
redundant_var_best      = array_redundant_var_best(find(array_power_best == power_best),:);
redundant_var_best      = redundant_var_best(1,:);
toc


% calculte the correspond trimmed variables
% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
cell_InitialStates      = {[0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                            [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                            [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                            [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
[x,~,exitflag,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                @Aerodynamics_trim_full_8var, ... 
                                cell_InitialStates, ...
                                options, ...
                                2, ...                              % LowerRotor.inteference
                                2, ...                              % UpperRotor.inteference
                                redundant_var_best(1), ...          % Prop.theta_0
                                redundant_var_best(2), ...          % Prop.isEnable
                                1, ...                              % Fus.isEnable
                                redundant_var_best(3), ...          % HorStab.delta_e
                                1, ...                              % HorStab.isEnable
                                redundant_var_best(4), ...          % VerStab.delta_r
                                1, ...                              % VerStab.isEnable
                                redundant_var_best(5), ...          % theta_1c_diff
                                redundant_var_best(6));             % theta_1s_diff
                            
 %% Search the best redundant variables under a specific velocity 3.2 differential evolution de
tic
Rotorcraft.DoubleRotorHelicopter.U         = 80; 
Rotorcraft.DoubleRotorHelicopter.V         = 0;
Rotorcraft.DoubleRotorHelicopter.W         = 0;
Rotorcraft.DoubleRotorHelicopter.U_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.V_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.W_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.p         = 0;
Rotorcraft.DoubleRotorHelicopter.q         = 0;
Rotorcraft.DoubleRotorHelicopter.r         = 0;
Rotorcraft.DoubleRotorHelicopter.p_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.q_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.r_dot     = 0;

% initial point for trimming
nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:9};
array_redundant_var_best        = [ones(1,6);0 0 0 0 0 0];
array_power_best                = zeros(2,1);

% if propeller is enabled
% 瀵绘�炬�?�瑙?
options.size_population     = 50;
options.max_generation      = 100;
options.scale_parameter     = 0.3;
options.crossover_proba     = 0.7;

problem             = struct;
problem.objective   = @(x) trim_power(Rotorcraft,nearest_initial_no_redundant,x(1),1,x(2),x(3),x(4),x(5));
problem.lb          = [0,deg2rad(-25),deg2rad(-30),deg2rad(-1),deg2rad(-1)];
problem.ub          = [deg2rad(40),deg2rad(25),deg2rad(30),deg2rad(1),deg2rad(4.5)];
problem.dimension   = 5;

[x,array_power_best(1)] = de(problem,options);
array_redundant_var_best(1,1) = x(1);
array_redundant_var_best(1,3:6) = x(2:5);


% if propeller is NOT enabled
options.size_population     = 50;
options.max_generation      = 1;
options.scale_parameter     = 0.3;
options.crossover_proba     = 0.7;

problem             = struct;
problem.objective   = @(x) trim_power(Rotorcraft,nearest_initial_no_redundant,0,0,x(1),x(2),x(3),x(4));
problem.lb          = [deg2rad(-25),deg2rad(-30),deg2rad(-1),deg2rad(-1)];
problem.ub          = [deg2rad(25),deg2rad(30),deg2rad(1),deg2rad(4.5)];
problem.dimension   = 4;

[array_redundant_var_best(2,3:6),array_power_best(2)] = de(problem,options);

% Search the global minimum among the local minimum results
power_best              = min(array_power_best);
redundant_var_best      = array_redundant_var_best(find(array_power_best == power_best),:);
redundant_var_best      = redundant_var_best(1,:);
toc


% calculte the correspond trimmed variables
% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
cell_InitialStates      = {[0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                            [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                            [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                            [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
[x,~,exitflag,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                @Aerodynamics_trim_full_8var, ... 
                                cell_InitialStates, ...
                                options, ...
                                2, ...                              % LowerRotor.inteference
                                2, ...                              % UpperRotor.inteference
                                redundant_var_best(1), ...          % Prop.theta_0
                                redundant_var_best(2), ...          % Prop.isEnable
                                1, ...                              % Fus.isEnable
                                redundant_var_best(3), ...          % HorStab.delta_e
                                1, ...                              % HorStab.isEnable
                                redundant_var_best(4), ...          % VerStab.delta_r
                                1, ...                              % VerStab.isEnable
                                redundant_var_best(5), ...          % theta_1c_diff
                                redundant_var_best(6));             % theta_1s_diff
                            
%% Search the best redundant variables under a specific velocity 3.2 adapting differential evolution jde
 tic
Rotorcraft.DoubleRotorHelicopter.U         = 48; 
Rotorcraft.DoubleRotorHelicopter.V         = 0;
Rotorcraft.DoubleRotorHelicopter.W         = 0;
Rotorcraft.DoubleRotorHelicopter.U_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.V_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.W_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.p         = 0;
Rotorcraft.DoubleRotorHelicopter.q         = 0;
Rotorcraft.DoubleRotorHelicopter.r         = 0;
Rotorcraft.DoubleRotorHelicopter.p_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.q_dot     = 0;
Rotorcraft.DoubleRotorHelicopter.r_dot     = 0;

% initial point for trimming
nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:9};
array_redundant_var_best        = [ones(1,6);0 0 0 0 0 0];
array_power_best                = zeros(2,1);

% if propeller is enabled
options.size_population     = 20;
options.max_generation      = 100;
options.scale_parameter     = 0.7;
options.scale_parameter_lb  = 0.1;
options.scale_parameter_ub  = 0.9;
options.scale_parameter_tau = 0.1;
options.crossover_proba     = 0.3;
options.crossover_proba_tau = 0.1;

problem             = struct;
problem.objective   = @(x) trim_power(Rotorcraft,nearest_initial_no_redundant,x(1),1,x(2),x(3),x(4),x(5));
problem.lb          = [0,deg2rad(-25),deg2rad(-30),deg2rad(-1),deg2rad(-1)];
problem.ub          = [deg2rad(40),deg2rad(25),deg2rad(30),deg2rad(1),deg2rad(4.5)];
problem.dimension   = 5;

[x,array_power_best(1),~] = jde_par(problem,options);
array_redundant_var_best(1,1) = x(1);
array_redundant_var_best(1,3:6) = x(2:5);
toc

% if propeller is NOT enabled
options.size_population     = 20;
options.max_generation      = 100;
options.scale_parameter     = 0.7;
options.scale_parameter_lb  = 0.1;
options.scale_parameter_ub  = 0.9;
options.scale_parameter_tau = 0.1;
options.crossover_proba     = 0.3;
options.crossover_proba_tau = 0.1;

problem             = struct;
problem.objective   = @(x) trim_power(Rotorcraft,nearest_initial_no_redundant,0,0,x(1),x(2),x(3),x(4));
problem.lb          = [deg2rad(-25),deg2rad(-30),deg2rad(-1),deg2rad(-1)];
problem.ub          = [deg2rad(25),deg2rad(30),deg2rad(1),deg2rad(4.5)];
problem.dimension   = 4;

[array_redundant_var_best(2,3:6),array_power_best(2),~] = jde_par(problem,options);

% Search the global minimum among the local minimum results
power_best              = min(array_power_best);
redundant_var_best      = array_redundant_var_best(find(array_power_best == power_best),:);
redundant_var_best      = redundant_var_best(1,:);
toc


% calculte the correspond trimmed variables
% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
cell_InitialStates      = {[0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                            [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                            [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                            [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
[x,~,exitflag,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                @Aerodynamics_trim_full_8var, ... 
                                cell_InitialStates, ...
                                options, ...
                                2, ...                              % LowerRotor.inteference
                                2, ...                              % UpperRotor.inteference
                                redundant_var_best(1), ...          % Prop.theta_0
                                redundant_var_best(2), ...          % Prop.isEnable
                                1, ...                              % Fus.isEnable
                                redundant_var_best(3), ...          % HorStab.delta_e
                                1, ...                              % HorStab.isEnable
                                redundant_var_best(4), ...          % VerStab.delta_r
                                1, ...                              % VerStab.isEnable
                                redundant_var_best(5), ...          % theta_1c_diff
                                redundant_var_best(6));             % theta_1s_diff