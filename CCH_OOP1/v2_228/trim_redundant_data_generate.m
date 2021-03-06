% 2.20 整机配平 带冗余变量
clear all
clc
h = 100;
[~,~,~,rho] = atmosisa(h);
table_trim_no_redundant_states = readtable('trim_result_no_redundant.csv');
table_trim_redundant_prop_states = readtable('trim_result_redundent_prop_fmincon_03_04_16_44.csv');
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
% 配平初始点
nearest_initial_no_redundant = table_trim_no_redundant_states{table_trim_no_redundant_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:9};
% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
cell_InitialStates      = {nearest_initial_no_redundant,[0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                            [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                            [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                            [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
[x,~,exitflag,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                @Aerodynamics_trim_full_8var, ... 
                                cell_InitialStates, ...
                                options, ...
                                2, ...                  % LowerRotor.inteference
                                2, ...                  % UpperRotor.inteference
                                deg2rad(15), ...         % Prop.theta_0
                                0, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                deg2rad(0), ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                deg2rad(0), ...          % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                deg2rad(0), ...         % theta_1c_diff
                                deg2rad(0));            % theta_1s_diff

%% 不同速度下,不同冗余变量下的配平
% 变量取值范围
U_min               = 0;
U_max               = 130;
Prop_theta_0_min    = 0;
Prop_theta_0_max    = deg2rad(36);
delta_e_min         = deg2rad(-25);
delta_e_max         = deg2rad(25);
delta_r_min         = deg2rad(-30);
delta_r_max         = deg2rad(30);
theta_1c_diff_min   = deg2rad(-1);
theta_1c_diff_max   = deg2rad(1);
theta_1s_diff_min   = 0;
theta_1s_diff_max   = deg2rad(4.5);

number_sample = 30000; % 样本数量
size_batch = 20; %并行池大小
number_batch = number_sample/size_batch; 
%number_sample_generated = 0; % 用于统计生产的样本数量
matrix_trim_states = zeros(number_sample,27);
%array_U = zeros(1,number_sample);
% Prop_theta_0,Prop_isEnable,delta_e,delta_r,theta_1c_diff,theta_1s_diff,U,theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2,v_01,v_02,beta_01,beta_1c1,beta_1s1,beta_02,beta_1c2,beta_1s2,power_total_LowerRotor,power_total_UpperRotor,power_total_Prop,power_total

disp('---------开始生成样本-----------')
disp(datetime)
bar = waitbar(0,'生成样本中...');

tic;
time_elapsed = 0;

for k = 1:number_batch
parfor j = 1+size_batch*(k-1):size_batch*k
    % 生成样本(冗余变量)
    U = unifrnd(U_min,U_max);
    Prop_theta_0 = unifrnd(Prop_theta_0_min,Prop_theta_0_max);
    Prop_isEnable = randi(2)-1;
    delta_e = unifrnd(delta_e_min,delta_e_max);
    delta_r = unifrnd(delta_r_min,delta_r_max);
    theta_1c_diff = unifrnd(theta_1c_diff_min,theta_1c_diff_max);
    theta_1s_diff = unifrnd(theta_1s_diff_min,theta_1s_diff_max);
    redundant_var = [Prop_theta_0, Prop_isEnable, delta_e, delta_r, theta_1c_diff, theta_1s_diff];
    
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
    
    if Prop_isEnable == 0
        nearest_initial_no_redundant = table_trim_no_redundant_states{table_trim_no_redundant_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:9};
    else
        nearest_initial_no_redundant = table_trim_redundant_prop_states{table_trim_redundant_prop_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:9};
    end
    
    % 获取相应配平变量
    % x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
    options                 = optimset('Display','off','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
    cell_InitialStates      = {nearest_initial_no_redundant,[0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                                [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                                [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                                [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
    
    [x,~,exitflag,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                    @Aerodynamics_trim_full_8var, ... 
                                    cell_InitialStates, ...
                                    options, ...
                                    2, ...                  % LowerRotor.inteference
                                    2, ...                  % UpperRotor.inteference
                                    Prop_theta_0, ...         % Prop.theta_0
                                    Prop_isEnable, ...                  % Prop.isEnable
                                    1, ...                  % Fus.isEnable
                                    delta_e, ...         % HorStab.delta_e
                                    1, ...                  % HorStab.isEnable
                                    delta_r, ...          % VerStab.delta_r
                                    1, ...                  % VerStab.isEnable
                                    theta_1c_diff, ...         % theta_1c_diff
                                    theta_1c_diff);            % theta_1s_diff
    if exitflag > 0
        matrix_trim_states(j,:) = [U ...
                                    x ...
                                    redundant_var ...
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
        matrix_trim_states(j,:) = [U redundant_var nan*ones(1,20)];
    end
end

    time_elapsed = toc;
    time_average = time_elapsed/k;
    time_estimated = time_average * (number_batch-k);
    if k < number_batch
        str=['生成样本中...',num2str(k*size_batch),'/',num2str(number_sample),'（已用时 ',num2str(time_elapsed),'s，预计 ',num2str(time_estimated), 's后完成)'];
    else
        str='已完成！';   
    end
    waitbar(k/number_batch,bar,str)                       % 更新进度条bar，配合bar使用
end

disp('---------样本生成完毕-----------')
disp(datetime)

%% 保存结果
VariableNames = {'U','theta_0','theta_diff','theta_1c','theta_1s','theta','phi','v_i1','v_i2', ...
                'Prop_theta_0','Prop_isEnable','theta_1c_diff','theta_1s_diff','delta_e','delta_r', ...
                'v_01','v_02', ...
                'beta_01','beta_1c1','beta_1s1','beta_02','beta_1c2','beta_1s2', ...
                'power_total_LowerRotor', 'power_total_UpperRotor', 'power_total_Prop' ,'power_total'};
table_trim_states = array2table(matrix_trim_states,'VariableNames',VariableNames);
filename = ['trim_redundant_data_generation_',datestr(now,'mm_dd_HH_MM'),'.csv'];
writetable(table_trim_states,filename);