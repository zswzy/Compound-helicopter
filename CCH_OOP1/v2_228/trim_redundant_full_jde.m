% 3.2 Rotorcraft trim, full redundant (6 redundatn variables)
clear all
clc
h = 100; % flight altitude
[~,~,~,rho] = atmosisa(h);
table_trim_no_redundant_states = readtable('trim_result_no_redundant.csv');
table_trim_redundant_prop_states = readtable('trim_result_redundent_prop_fmincon_03_04_16_44.csv');
%% build object
run init_build.m

%% Search the best redundant variables under a specific velocity 3.2 adapting differential evolution jde

number_of_U         = 131;
array_U             = linspace(0,130,number_of_U);
matrix_trim_states  = zeros(number_of_U,27);
% U,theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2,Prop_theta_0,Prop_isEnable,theta_1c_diff,theta_1s_diff,delta_e,delta_r,v_01,v_02,beta_01,beta_1c1,beta_1s1,beta_02,beta_1c2,beta_1s2,power_total_LowerRotor,power_total_UpperRotor,power_total_Prop,power_total
disp('---------开始迭代求解-----------')
disp(datetime)
bar = waitbar(0,'迭代求解中...');
tic;
time_elapsed = 0;

for j = 1:number_of_U
    disp(['U = ', num2str(array_U(j))])
    % struct
    Rotorcraft = struct;
    Rotorcraft.DoubleRotorHelicopter    = DoubleRotorHelicopter;
    Rotorcraft.LowerRotor               = LowerRotor;
    Rotorcraft.UpperRotor               = UpperRotor;
    Rotorcraft.Prop                     = Prop;
    Rotorcraft.Fus                      = Fus;
    Rotorcraft.HorStab                  = HorStab;  
    Rotorcraft.VerStab                  = VerStab;
    % flight speed

    Rotorcraft.DoubleRotorHelicopter.U         = array_U(j);
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

    array_redundant_var_best        = [ones(1,6);0 0 0 0 0 0];
    array_power_best                = ones(2,1)*inf;

    % jde options 
    options.size_population     = 30;
    options.max_generation      = 100;
    options.scale_parameter     = 0.7;
    options.scale_parameter_lb  = 0.1;
    options.scale_parameter_ub  = 0.9;
    options.scale_parameter_tau = 0.1;
    options.crossover_proba     = 0.3;
    options.crossover_proba_tau = 0.1;
    
    % initial point for trimming
    nearest_initial_no_redundant = table_trim_redundant_prop_states{table_trim_redundant_prop_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:9};
    % if propeller is enabled
    problem             = struct;
    problem.objective   = @(x) trim_power(Rotorcraft,nearest_initial_no_redundant,x(1),1,x(2),x(3),x(4),x(5));
    problem.lb          = [0,deg2rad(-25),deg2rad(-30),deg2rad(-1),deg2rad(-1)];
    problem.ub          = [deg2rad(40),deg2rad(25),deg2rad(30),deg2rad(1),deg2rad(4.5)];
    problem.dimension   = 5;

    [x,array_power_best(1),~] = jde(problem,options);
    array_redundant_var_best(1,1) = x(1);
    array_redundant_var_best(1,3:6) = x(2:5);

    % initial point for trimming
    nearest_initial_no_redundant = table_trim_no_redundant_states{table_trim_no_redundant_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:9};
    % if propeller is NOT enabled
    if array_U(j) <= 50
        problem             = struct;
        problem.objective   = @(x) trim_power(Rotorcraft,nearest_initial_no_redundant,0,0,x(1),x(2),x(3),x(4));
        problem.lb          = [deg2rad(-25),deg2rad(-30),deg2rad(-1),deg2rad(-1)];
        problem.ub          = [deg2rad(25),deg2rad(30),deg2rad(1),deg2rad(4.5)];
        problem.dimension   = 4;

        [array_redundant_var_best(2,3:6),array_power_best(2),~] = jde(problem,options);
    end
        
    % Search the global minimum among the local minimum results
    power_best              = min(array_power_best);
    redundant_var_best      = array_redundant_var_best(find(array_power_best == power_best),:);
    redundant_var_best      = redundant_var_best(1,:);

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
     if exitflag > 0
        matrix_trim_states(j,:) = [array_U(j) ...
                                    x ...
                                    redundant_var_best ... 
                                    Rotorcraft.LowerRotor.v_0 ...
                                    Rotorcraft.UpperRotor.v_0 ...
                                    Rotorcraft.LowerRotor.beta_0 ...
                                    Rotorcraft.LowerRotor.beta_1c ...
                                    Rotorcraft.LowerRotor.beta_1s ...
                                    Rotorcraft.UpperRotor.beta_0 ...
                                    Rotorcraft.UpperRotor.beta_1c ...
                                    Rotorcraft.UpperRotor.beta_1s ...
                                    Rotorcraft.LowerRotor.Power_total ...
                                    Rotorcraft.UpperRotor.Power_total ...
                                    Rotorcraft.Prop.Power_total ...
                                    power_total];
    else
        matrix_trim_states(j,:) = [array_U(j) nan*ones(1,26)];
     end
    
    time_elapsed = toc;
    time_average = time_elapsed/j;
    time_estimated = time_average * (number_of_U-j);
    if j < number_of_U
        str=['迭代求解中...',num2str(j),'/',num2str(number_of_U),'（已用时 ',num2str(time_elapsed),'s，预计 ',num2str(time_estimated), 's后完成)'];
    else
        str='已完成！';   
    end
    waitbar(j/number_of_U,bar,str)                       % 更新进度条bar，配合bar使用
    
end
disp('---------完成迭代求解-----------')
disp(datetime)

%% 保存结果
VariableNames = {'U','theta_0','theta_diff','theta_1c','theta_1s','theta','phi','v_i1','v_i2', ...
                'Prop_theta_0','Prop_isEnable','theta_1c_diff','theta_1s_diff','delta_e','delta_r', ...
                'v_01','v_02', ...
                'beta_01','beta_1c1','beta_1s1','beta_02','beta_1c2','beta_1s2', ...
                'power_total_LowerRotor', 'power_total_UpperRotor', 'power_total_Prop' ,'power_total'};
table_trim_states = array2table(matrix_trim_states,'VariableNames',VariableNames);
filename = ['trim_result_redundent_full_jde_',datestr(now,'mm_dd_HH_MM'),'.csv'];
writetable(table_trim_states,filename);