% 2.28 整机配平 带冗余变量 只有尾推 simple方法
clear all
clc
h = 100;
[~,~,~,rho] = atmosisa(h);
table_trim_states = readtable('trim_result_no_redundant.csv');
%% 建立对象
run init_build.m

%% 悬停/前飞配平 X,Y,Z,L,M,N=0,[theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
Rotorcraft.DoubleRotorHelicopter.U         = 40;
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

% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',30,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
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
                                deg2rad(15), ...        % Prop.theta_0
                                0, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                deg2rad(0), ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                deg2rad(0), ...         % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                deg2rad(0), ...         % theta_1c_diff
                                deg2rad(0));            % theta_1s_diff
 
%% 不同速度下的配平

array_U = 0:100;
[~,number_of_U] = size(array_U);
matrix_trim_states = zeros(number_of_U,27);
redundant_var_best = [0,0];
% U,theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2,Prop_theta_0,Prop_isEnable,delta_e,delta_r,theta_1c_diff,theta_1s_diff,v_01,v_02,beta_01,beta_1c1,beta_1s1,beta_02,beta_1c2,beta_1s2,power_total_LowerRotor,power_total_UpperRotor,power_total_Prop,power_total
disp('---------开始迭代求解-----------')
disp(datetime)
bar = waitbar(0,'迭代求解中...');
tic;
time_elapsed = 0;

parfor j = 1:number_of_U
    disp(['U = ', num2str(array_U(j))])
    % 建立结构体(并行计算需要）
    Rotorcraft = struct;
    Rotorcraft.DoubleRotorHelicopter    = DoubleRotorHelicopter;
    Rotorcraft.LowerRotor               = LowerRotor;
    Rotorcraft.UpperRotor               = UpperRotor;
    Rotorcraft.Prop                     = Prop;
    Rotorcraft.Fus                      = Fus;
    Rotorcraft.HorStab                  = HorStab;  
    Rotorcraft.VerStab                  = VerStab;
    % 初始化前飞速度
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
    
    % 配平初始点
    nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:9};
    
    array_redundant_var_best        = [1 1;0 0];
    array_power_best                = [0;0];

    % 若有尾推
    array_Prop_theta_0  = linspace(deg2rad(0),deg2rad(40),200);
    array_power         = zeros(size(array_Prop_theta_0));

    x_trim_last = nearest_initial_no_redundant;
    for k = 1:200
        ele_Prop_theta_0 = array_Prop_theta_0(k);
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
        array_power(k) = power_total;
        
    end
    array_power_best(1) = min(array_power);
    array_redundant_var_best(1,1) = array_Prop_theta_0(1,array_power == array_power_best(1));

    % 若无尾推
    array_power_best(2) = trim_power(Rotorcraft,nearest_initial_no_redundant,0,0);

    % 寻找全局最优解
    power_best              = min(array_power_best);
    redundant_var_best      = array_redundant_var_best(find(array_power_best == power_best),:);

    % 获取相应配平变量
    % x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
    options                 = optimset('Display','off','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
    cell_InitialStates      = {nearest_initial_no_redundant, [0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
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
                                    deg2rad(0), ...                     % HorStab.delta_e
                                    1, ...                              % HorStab.isEnable
                                    deg2rad(0), ...                     % VerStab.delta_r
                                    1, ...                              % VerStab.isEnable
                                    deg2rad(0), ...                     % theta_1c_diff
                                    deg2rad(0));                        % theta_1s_diff
                                
    if exitflag > 0
        matrix_trim_states(j,:) = [array_U(j) ...
                                    x ...
                                    redundant_var_best ... 
                                    0 0 0 0 ...
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
                'Prop_theta_0','Prop_isEnable','delta_e','delta_r','theta_1c_diff','theta_1s_diff', ...
                'v_01','v_02', ...
                'beta_01','beta_1c1','beta_1s1','beta_02','beta_1c2','beta_1s2', ...
                'power_total_LowerRotor', 'power_total_UpperRotor', 'power_total_Prop' ,'power_total'};
table_trim_states = array2table(matrix_trim_states,'VariableNames',VariableNames);
% filename = ['trim_result_redundent_prop_fmincon_',datestr(now,'mm_dd_HH_MM'),'.csv'];
filename = 'trim_result_redundent_prop_simple.csv';
writetable(table_trim_states,filename);

