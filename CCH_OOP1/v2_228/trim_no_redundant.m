% 2.19 整机配平 无冗余变量
% 生成全包线飞行配平数据 trim_result_no_redundant.csv 并画图
clear all
clc
h = 100;
[~,~,~,rho] = atmosisa(h);

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
                                deg2rad(0), ...         % Prop.theta_0
                                0, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                deg2rad(0), ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                deg2rad(0), ...         % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                deg2rad(0), ...         % theta_1c_diff
                                deg2rad(0));            % theta_1s_diff
                            
%% 不同速度下的配平
number_of_U = 131;
array_U = linspace(0,130,number_of_U);
matrix_trim_states = zeros(number_of_U,27);
% U,theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2,v_01,v_02,beta_01,beta_1c1,beta_1s1,beta_02,beta_1c2,beta_1s2,power_total_LowerRotor,power_total_UpperRotor,power_total_Prop,power_total
x_trim_last = [0.01,0,0,0,0,0,10,10]; % 上一次的解
for j = 1:number_of_U
    disp(array_U(j))
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

    % x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
    options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
    cell_InitialStates      = {x_trim_last,[0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                                [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                                [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                                [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
    [x_trim,~,exitflag,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                    @Aerodynamics_trim_full_8var, ... 
                                    cell_InitialStates, ...
                                    options, ...
                                    2, ...                  % LowerRotor.inteference
                                    2, ...                  % UpperRotor.inteference
                                    deg2rad(0), ...         % Prop.theta_0
                                    0, ...                  % Prop.isEnable
                                    1, ...                  % Fus.isEnable
                                    deg2rad(0), ...         % HorStab.delta_e
                                    1, ...                  % HorStab.isEnable
                                    deg2rad(0), ...          % VerStab.delta_r
                                    1, ...                  % VerStab.isEnable
                                    deg2rad(0), ...         % theta_1c_diff
                                    deg2rad(0));            % theta_1s_diff
    if exitflag > 0
        matrix_trim_states(j,:) = [array_U(j) ...
                                    x_trim ...
                                    0 0 0 0 0 0 ...
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
                                    Rotorcraft.Prop.Power_resist ...
                                    power_total];
        x_trim_last = x_trim;
    else
        matrix_trim_states(j,:) = [array_U(j) nan*ones(1,20)];
    end
end

%% 保存结果
VariableNames = {'U','theta_0','theta_diff','theta_1c','theta_1s','theta','phi','v_i1','v_i2', ...
                'Prop_theta_0','Prop_isEnable','theta_1c_diff','theta_1s_diff','delta_e','delta_r', ...
                'v_01','v_02', ...
                'beta_01','beta_1c1','beta_1s1','beta_02','beta_1c2','beta_1s2', ...
                'power_total_LowerRotor', 'power_total_UpperRotor', 'power_total_Prop' ,'power_total'};
table_trim_states = array2table(matrix_trim_states,'VariableNames',VariableNames);
writetable(table_trim_states,'trim_result_no_redundant.csv');

%% 可视化
table_trim_states = readtable('trim_result_no_redundant.csv');
array_U             = table_trim_states.U;
array_theta_0       = table_trim_states.theta_0;
array_theta_diff    = table_trim_states.theta_diff;
array_theta_1c      = table_trim_states.theta_1c;
array_theta_1s      = table_trim_states.theta_1s;
array_v_01          = table_trim_states.v_01;
array_v_02          = table_trim_states.v_02;
array_theta         = table_trim_states.theta;
array_phi           = table_trim_states.phi;
array_beta_01       = table_trim_states.beta_01;
array_beta_1c1      = table_trim_states.beta_1c1;
array_beta_1s1      = table_trim_states.beta_1s1;
array_beta_02       = table_trim_states.beta_02;
array_beta_1c2      = table_trim_states.beta_1c2;
array_beta_1s2      = table_trim_states.beta_1s2;
array_power_total_LowerRotor    = table_trim_states.power_total_LowerRotor;
array_power_total_UpperRotor    = table_trim_states.power_total_UpperRotor;
array_power_total_Prop          = table_trim_states.power_total_Prop;
array_power_total               = table_trim_states.power_total;

figure(1)
subplot(1,2,1)
plot(array_U,rad2deg(array_theta),'linewidth',1.5)
hold on
plot([100 100], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\theta(deg)'); title('U-\theta'); grid on;
subplot(1,2,2)
plot(array_U,rad2deg(array_phi),'linewidth',1.5)
hold on
plot([100 100], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\phi(deg)'); title('U-\phi'); grid on;

figure(2)
subplot(2,2,1)
plot(array_U,rad2deg(array_theta_0),'linewidth',1.5)
hold on
plot([100 100], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\theta_0 (deg)'); title('U-\theta_0'); grid on;
subplot(2,2,2)
plot(array_U,rad2deg(array_theta_diff),'linewidth',1.5)
hold on
plot([100 100], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\theta_{diff} (deg)'); title('U-\theta_{diff}'); grid on;
subplot(2,2,3)
plot(array_U,rad2deg(array_theta_1c),'linewidth',1.5)
hold on
plot([100 100], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\theta_{1c} (deg)'); title('U-\theta_{1c}'); grid on;
subplot(2,2,4)
plot(array_U,rad2deg(array_theta_1s),'linewidth',1.5)
hold on
plot([100 100], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\theta_{1s} (deg)'); title('U-\theta_{1s}'); grid on;

figure(3)
plot(array_U,array_v_01,'linewidth',1.5)
hold on
plot(array_U,array_v_02,'linewidth',1.5)
hold on
plot([100 100], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
legend('v_{01}','v_{02}','Location','best');
xlabel('U'); ylabel('v_0 (m/s)'); title('U-v_0'); grid on;

figure(4)
plot(array_U,rad2deg(array_beta_01),'linewidth',1.5)
hold on 
plot(array_U,rad2deg(array_beta_02),'linewidth',1.5)
hold on
plot([100 100], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
legend('\beta_{01}','\beta_{02}','Location','best')
xlabel('U'); ylabel('\beta_{01},\beta_{02}(deg)'); title('U-\beta_0'); grid on;

figure(5)
subplot(1,2,1)
plot(array_U,rad2deg(array_beta_1c1),'linewidth',1.5)
hold on 
plot(array_U,rad2deg(array_beta_1c2),'linewidth',1.5)
hold on
plot([100 100], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
legend('\beta_{1c1}','\beta_{1c2}','Location','best')
xlabel('U'); ylabel('\beta_{1c1},\beta_{1c2}(deg)'); title('U-\beta_{1c}'); grid on;
subplot(1,2,2)
plot(array_U,rad2deg(array_beta_1s1),'linewidth',1.5)
hold on 
plot(array_U,rad2deg(array_beta_1s2),'linewidth',1.5)
hold on
plot([100 100], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
legend('\beta_{1s1}','\beta_{1s2}','Location','best')
xlabel('U'); ylabel('\beta_{1s1},\beta_{1s2}(deg)'); title('U-\beta_{1s}'); grid on;

figure(6)
plot(array_U,array_power_total_LowerRotor/1000,'linewidth',1.5)
hold on
plot(array_U,array_power_total_UpperRotor/1000,'linewidth',1.5)
hold on
plot(array_U,array_power_total/1000,'linewidth',1.5)
hold on
plot([100 100], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
legend('Power_{lower rotor}','Power_{upper rotor}','Power_{total}','Location','best');
xlabel('U'); ylabel('Power (kW)'); title('U-Power required'); grid on;