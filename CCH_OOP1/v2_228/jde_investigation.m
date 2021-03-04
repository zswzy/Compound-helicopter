%% 3.2 研究不同的超参数对性能的影响
% 首先研究不同的种群规模对结果以及鲁棒性的影响
clear all
clc
time_start = datetime;

array_population_size       = 4:4:30;
[~,number_population_size]  = size(array_population_size);
matrix_power_best           = zeros(number_population_size,8);
matrix_time_elapsed         = zeros(number_population_size,8);
matrix_max_generation       = zeros(number_population_size,8);

disp(datetime)
bar = waitbar(0,'迭代求解中...');
tic;
time_elapsed = 0;

for k = 1:number_population_size
    disp(array_population_size(k))
    % 每个population size 运行八次，分析结果是否一致
    for j = 1:8
        [power_best,redundant_var_best,output] = de_parameter(80,array_population_size(k));
        
        matrix_power_best(k,j) = power_best;
        matrix_time_elapsed(k,j) = output.time_elapsed;
        matrix_max_generation(k,j) = output.max_generation;
        
        time_elapsed = toc;
        time_average = time_elapsed/(j*k);
        time_estimated = time_average * (number_population_size*8 - j*k);
        if j*k < number_population_size*8
            str=['迭代求解中...',num2str(8*(k-1)+j),'/',num2str(number_population_size*8),'（已用时 ',num2str(time_elapsed),'s，预计 ',num2str(time_estimated), 's后完成)'];
        else
            str='已完成！';   
        end
        waitbar((8*(k-1)+j)/(number_population_size*8),bar,str)                       % 更新进度条bar，配合bar使用
    end
end

time_end = datetime;
%%
filename = ['jde_investigation/jde_investigation3_',datestr(datetime),'.mat'];
save(filename)
%%
plot(array_population_size,matrix_time_elapsed(:,1))

function [power_best,redundant_var_best,output] = de_parameter(U,size_population)

h = 100; % 高度
[~,~,~,rho] = atmosisa(h);
table_trim_states = readtable('trim_result_no_redundant.csv');
% 建立对象
run init_build.m
% 寻找在某一配平速度下的最优冗余变量 3.2 进化差分 jde

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

[x,array_power_best(1),output] = jde(problem,options);
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