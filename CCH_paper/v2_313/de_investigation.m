% 研究不同的超参数对性能的影响

[power_best,output] = de_parameter(80,10,0.5,0.5);
time_elapsed = output.time_elapsed;
max_generation = output.max_generation;

function [power_best,output] = de_parameter(U,size_population,scale_parameter,crossover_proba)

h = 100; % 高度
[~,~,~,rho] = atmosisa(h);
table_trim_states = readtable('trim_result_no_redundant.csv');
%% 建立对象
run init_build.m
%% 寻找在某一配平速度下的最优冗余变量 2.28 进化差分 de

Rotorcraft.DoubleRotorHelicopter.U         = U; 
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

% 配平初始点
nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:9};
array_redundant_var_best        = [ones(1,6);0 0 0 0 0 0];
array_power_best                = ones(2,1)*inf;

% 若有尾推
% 寻找最优解
options.size_population     = size_population;
options.max_generation      = 100;
options.scale_parameter     = scale_parameter;
options.crossover_proba     = crossover_proba;

problem             = struct;
problem.objective   = @(x) trim_power(Rotorcraft,nearest_initial_no_redundant,x(1),1,x(2),x(3),x(4),x(5));
problem.lb          = [0,deg2rad(-25),deg2rad(-30),deg2rad(-1),deg2rad(-1)];
problem.ub          = [deg2rad(40),deg2rad(25),deg2rad(30),deg2rad(1),deg2rad(4.5)];
problem.dimension   = 5;


[x,array_power_best(1),output] = de(problem,options);
array_redundant_var_best(1,1) = x(1);
array_redundant_var_best(1,3:6) = x(2:5);


% 寻找全局最优解
power_best              = min(array_power_best);
redundant_var_best      = array_redundant_var_best(find(array_power_best == power_best),:);
redundant_var_best      = redundant_var_best(1,:);


% 获取相应配平变量
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
                                2, ...                  % LowerRotor.inteference
                                2, ...                  % UpperRotor.inteference
                                redundant_var_best(1), ...         % Prop.theta_0
                                redundant_var_best(2), ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                redundant_var_best(3), ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                redundant_var_best(4), ...          % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                redundant_var_best(5), ...         % theta_1c_diff
                                redundant_var_best(6));            % theta_1s_diff
                            

end