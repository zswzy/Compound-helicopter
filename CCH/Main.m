%% Main file
clear all
clc

%% 定义基本参数
global Heli% 直升机结构体，内含运动状态
global Control % 操纵变量结构体，内含操纵变量
% 飞行高度(海平面）
global h
h = 10;

% 大气数据
global rho
[~,~,~,rho] = atmosisa(h); %温度，音速，压强，密度
% 重力加速度
global g
g = 9.81;

global Rotor b m_b z_diff R A Omega s theta_t omega_n I_beta a_0 delta c K_beta gamma_M  e M_beta gamma_s x_H1 y_H1 z_H1 x_H2 y_H2 z_H2 h_R1 h_R2 x_cg
% 结构体Rotor
Rotor.C_Z=0; Rotor.C_Q0=0;
% 上下旋翼各自桨叶数
b = 3;
% 桨叶质量
m_b = 60;
% 上下旋翼间距 m
z_diff = 0.77;
% 主旋翼半径 m
R = 5.49;
%桨盘面积
A = pi*R^2;
% 主旋翼转速rad/s
Omega = 35.9;
% 主旋翼实度
s = 0.127;
% 主旋翼桨叶扭转角 rad
theta_t = deg2rad(-10);
% 主旋翼一阶挥舞固有频率 
omega_n = 1.4*Omega;
% 主旋翼挥舞惯性矩 kg m^2
I_beta = 450;
% 主旋翼升力线斜率,NACA0012
a_0 = 5.7;
%主旋翼桨叶阻力系数
delta = 0.008;
% 主旋翼桨叶弦长m
c = 0.29;
% K_beta 弹簧刚度  Nm/rad
% K_beta = 220500;
K_beta = 183397;
% 主旋翼洛克数 
gamma_M = 5.41;
% 无量纲等效铰偏置量epsilon 0.4左右,e为带量纲的
% epsilon = 2*((omega_n/Omega)^2-1)/(1+2*(omega_n/Omega)^2);
% e = R*epsilon;
%epsilon = e/R;
e = 2.58/R;
% 主旋翼对挥舞铰的质量静矩 200多
% M_beta = 1/2*R^2*m_b/R; 
% M_beta = I_beta*3/2*(1/R)/(1-epsilon);
M_beta = 123;
% 主旋翼桨毂纵向安装角
gamma_s = deg2rad(3);
%gamma_s = deg2rad(0);
% 主旋翼桨毂横向安装角
i_phi = 0;
% 主旋翼位置 1：下旋翼，2：上旋翼
h_R1 = 0.89;
h_R2 = 0.89+z_diff;
x_cg = 0;
x_H1 = 0;
y_H1 = 0;
z_H1 = -0.89;
x_H2 = 0;
y_H2 = 0;
z_H2 = z_H1-z_diff;


global Propeller R_PR A_PR Omega_PR s_PR theta_tPR  a_PR delta_PR x_PR y_PR z_PR K_PR I_beta_PR c_PR kappa
% 推进桨半径 m
R_PR = 1.3;
% 推进桨桨盘面积
A_PR = R_PR^2*pi;
% 推进桨转速 rad/s 162
Omega_PR = 162;
% 推进桨实度
s_PR = 0.2;
% 推进桨叶扭转
theta_tPR = deg2rad(-30);
% 推进桨叶升力线斜率,NACA0012
a_PR = 5.7;
% 推进桨叶阻力系数
delta_PR = 0.008;
% 推进桨叶坐标
x_PR = -7.66;
y_PR = 0;
z_PR = 0;
% 动压损失系数
K_PR=0.85;
% 挥舞惯性矩 kg/m^2
I_beta_PR = 21;
% 推进桨桨叶弦长
c_PR=0.14;
% 叶尖损失系数
kappa = 1;

global GW Ixx Iyy Izz 
% 整机质量 kg
GW = 5500;
%转动惯量
Ixx = 5518;
Iyy = 26844;
Izz = 23048;

global x_F y_F z_F l_F s_F Fuselage
% 机身气动中心
x_F = 0;
y_F = 0;
z_F = 0;
% 机身长度
l_F = 12;
% 机身最大迎风面积
s_F = 2.4*10;

global alpha_0HS s_H s_e xi_H x_HS y_HS z_HS Tail
% 平尾安装角
alpha_0HS = 0;
% 平尾面积
s_H = 5.6;
% 升降舵面积
s_e = 0.3*s_H;
% 平尾后掠角
xi_H = 0;
% 平尾位置
x_HS = -6.8;
y_HS = 0;
z_HS = 0.2;

global alpha_0VS x_VS y_VS z_VS s_r xi_V s_V Fin
% 垂尾安装角
alpha_0VS = 0;
% 垂尾位置
x_VS = -6.8;
y_VS = 0;
z_VS = -0.5;
% 垂尾后掠角
xi_V = 0;
% 垂尾面积
s_V = 2.79;
% 垂尾舵面面积
s_r = 0.3*s_V;

global L_BH
% 转换矩阵
L_BH = [cos(gamma_s)    0     -sin(gamma_s);
        0               1     0;
        sin(gamma_s)    0     cos(gamma_s)];

%% 1.15 下旋翼测试
% global theta_0
% theta_0 = deg2rad(10);
% 
% InitialStates = 10; %v_0, beta_0, beta_1c, beta_1s
% 
% options=optimset('Display','iter','TolFun',1e-5,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
% [x,fval,exitflag,~] = fsolve(@SolveRotor,InitialStates,options);
% disp(['beta_0(deg):' num2str(rad2deg(Rotor.beta_0))])
% 
% clf
% clear Rotor
% global Rotor

%% 1.15 得到升力-扭矩曲线
% C_T_list=[]; C_Q_list=[]; T_list=[]; v_0_list = [];
% for theta_0 = linspace(deg2rad(5),deg2rad(15),50)
%     InitialStates = 10; %v_0, beta_0, beta_1c, beta_1s
%     options=optimset('Display','off','TolFun',1e-8,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
%     [x,fval,exitflag,~] = fsolve(@SolveRotor,InitialStates,options);
%     if exitflag>0
%         C_T_list = [C_T_list Rotor.C_Z];
%         C_Q_list = [C_Q_list Rotor.C_Q0];
%         T_list = [T_list Rotor.Z];
%         v_0_list = [v_0_list Rotor.v_0];
%     end
%     
% end
% 
% plot(abs(C_Q_list) ,abs(C_T_list) )
% xlabel('C_Q')
% ylabel('C_T')
% %plot(abs(T_list))
% grid

% %% 1.16 下旋翼测试
% global theta_01 Rotor
% theta_01 = deg2rad(13);
% InitialStates = 10; %v_0
% options=optimset('Display','iter','TolFun',1e-5,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
% [x,fval,exitflag,~] = fsolve(@CoaxialRotor,InitialStates,options);
% disp(Rotor)
% 

%% 1.16 共轴旋翼测试 
global theta_01 theta_02 Rotor
theta_01 = deg2rad(10);
theta_02 = theta_01;
InitialStates = [10,10] ; %v_01,v_02
options=optimset('Display','iter','TolFun',1e-8,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
[x,fval,exitflag,~] = fsolve(@CoaxialRotor,InitialStates,options);
disp(Rotor)

%% 1.16 共轴旋翼测试 设定总力 计算总距 
% global  Rotor

% InitialStates = [10,10,deg2rad(10),0,0] ; %v_01,v_02，theta_0,theta_1c,theta_1s
% options=optimset('Display','iter','TolFun',1e-8,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
% [x,fval,exitflag,~] = fsolve(@CoaxialRotor,InitialStates,options);
% disp(Rotor)

%% 1.17 加入尾翼 
% % 姿态角 phi,theta 需要求解
% Heli.psi = deg2rad(0); 
% 
% % 运动状态
% Heli.V = 0; %悬停,或平直飞行
% 
% Heli.u_dot = 0;
% Heli.v_dot = 0;
% Heli.w_dot = 0;
% Heli.p = 0;
% Heli.q = 0;
% Heli.r = 0;
% Heli.p_dot = 0;
% Heli.q_dot = 0;
% Heli.r_dot = 0;
% 
% InitialStates = [10,10,deg2rad(10),0,0,0,0,deg2rad(0)] ; %v_01,v_02，theta_0,theta_diff,theta_1c,theta_1s,phi,theta
% options=optimset('Display','iter','TolFun',1e-8,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
% [x,fval,exitflag,~] = fsolve(@Aerodynamics,InitialStates,options);
% if exitflag > 0
%    CalculatePower() 
%    varNames = {'v_01','v_02','theta_0(deg)','theta_diff(deg)','theta_1c(deg)','theta_1s(deg)','phi(deg)','theta(deg)','Power1(kW)','Power2(kW)','Power(kW)'};
%    varTypes = {'double','double','double','double','double','double','double','double','double','double','double'};
%    Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
%    Tvar(1,:) = {x(1) x(2) rad2deg(x(3)) rad2deg(x(4)) rad2deg(x(5)) rad2deg(x(6)) rad2deg(x(7)) rad2deg(x(8)) Rotor.Power1/1000 Rotor.Power2/1000 Rotor.Power/1000};
%    disp(['V: ' num2str(Heli.V)])
%    disp(Tvar)
% end

%% 1.17 Aerodynamics 尝试迭代求解不同v对应的速度，姿态和控制 无气动干扰，无尾推，无舵面,半差动
% InitialStates = [10,10,deg2rad(10),0,0,0,0,deg2rad(-0.1)];
% TrimmedVariables = InitialStates;
% aPower = [];
% atheta = [];
% atheta_0 = [];
% atheta_1s = [];
% atheta_1c = [];
% atheta_diff = [];
% 
% Vrange = 1:1:90;
% for V  = Vrange
%     InitialStates = TrimmedVariables;
%     Heli.V = V;
%     Heli.u_dot = 0;
%     Heli.v_dot = 0;
%     Heli.w_dot = 0;
%     Heli.p = 0;
%     Heli.q = 0;
%     Heli.r = 0;
%     Heli.p_dot = 0;
%     Heli.q_dot = 0;
%     Heli.r_dot = 0;
%     options=optimset('Display','off','TolFun',1e-8,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
%     [TrimmedVariables,fval,exitflag,~] = fsolve(@Aerodynamics,InitialStates,options);
%     if exitflag > 0
%        CalculatePower() 
%        aPower = [aPower Rotor.Power];
%        atheta = [atheta TrimmedVariables(8)];
%        atheta_0 = [atheta_0 TrimmedVariables(3)];
%        atheta_1s = [atheta_1s TrimmedVariables(4)];
%        atheta_1c = [atheta_1c TrimmedVariables(5)];
%        atheta_diff = [atheta_diff TrimmedVariables(6)];
%     else
%        disp(V)
%     end
% end
% 
% figure(1)
% plot(Vrange./(Omega*R),rad2deg(atheta),'linewidth',2)
% xlabel('\mu'); ylabel('\theta(deg)');
% grid;
% 
% figure(2)
% plot(Vrange./(Omega*R),aPower./746,'linewidth',2)
% xlabel('\mu'); ylabel('Power(HP)');
% grid;
% 
% figure(3)
% plot(Vrange./(Omega*R),rad2deg(atheta_0),'linewidth',2)
% xlabel('\mu'); ylabel('\theta_0(deg)');
% grid;

%% 1.18 AerodynamicsV2 基础配平测试，无舵面，有尾推，平直平飞/悬停，不含不含部件间气动干扰以及旋翼间干扰，半差动
% 尾推采用曹燕论文拟合数据
% 俯仰角为冗余变量，需要预设，并且会对应同一配平状态的不同功率
Heli.psi = deg2rad(0); 
% 冗余变量
Heli.theta = deg2rad(0); %预设值

% 运动状态
Heli.V = 80; %悬停,或平直飞行
Heli.u_dot = 0;
Heli.v_dot = 0;
Heli.w_dot = 0;
Heli.p = 0;
Heli.q = 0;
Heli.r = 0;
Heli.p_dot = 0;
Heli.q_dot = 0;
Heli.r_dot = 0;

InitialStates = [10,10,deg2rad(10),0,0,0,deg2rad(0),deg2rad(0)] ; %v_01,v_02，theta_0,theta_diff,theta_1c,theta_1s,theta_PR,phi
options=optimset('Display','iter','TolFun',1e-8,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
[x,fval,exitflag,~] = fsolve(@AerodynamicsV2,InitialStates,options);
if exitflag > 0
   CalculatePowerV2() 
   varNames = {'v_01','v_02','theta_0(deg)','theta_diff(deg)','theta_1c(deg)','theta_1s(deg)','theta_PR(deg)','Propeller Force(N)','phi(deg)','theta(deg)'};
   varTypes = {'double','double','double','double','double','double','double','double','double','double'};
   Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
   Tvar(1,:) = {Rotor.v_01 Rotor.v_02 rad2deg(Control.theta_0) rad2deg(Control.theta_diff) rad2deg(Control.theta_1c1) rad2deg(Control.theta_1s1) rad2deg(Control.theta_0PR) Propeller.X_PR rad2deg(Heli.phi) rad2deg(Heli.theta) };
   disp("################################ This is the test for AerodynamicsV2.")
   disp(['V: ' num2str(Heli.V)])
   disp(Tvar)
   
   varNames = {'Power1(kW)','Power2(kW)','PowerPR(kW)','PowerP(kW)','TotalPower(kW)'};
   varTypes = {'double','double','double','double','double'};
   Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
   Tvar(1,:) = {Rotor.Power1/1000 Rotor.Power2/1000 Propeller.Power/1000 Heli.PowerP/1000 Heli.Power/1000};
   disp(Tvar)
end

%% 1.18 AerodynamicsV2 迭代配平测试，无舵面，有尾推，平直平飞/悬停，不含不含部件间气动干扰以及旋翼间干扰，半差动
% % 尾推采用曹燕论文拟合数据
% % 俯仰角为冗余变量，需要预设，并且会对应同一配平状态的不同功率
% 
% % 优化配平测试。寻找某一平飞速度下的最优功率和俯仰角。
% VRange = 0:3:100; iV = 1; %iterator
% thetaBest = zeros(size(VRange));
% PowerMin = zeros(size(VRange));
% 
% disp('######################## Starting Optimization ########################')
% for V = VRange
%    
%     thetaList = deg2rad(-15:1:5);
%     PowerList = zeros(size(thetaList));
%     itheta = 1; %iterator
%     
%     for theta = thetaList %预设值
%         
%         Heli.theta = theta;
%         Heli.psi = deg2rad(0); 
%         
%         % 运动状态
%         Heli.V = V; %悬停,或平直飞行
%         Heli.u_dot = 0;
%         Heli.v_dot = 0;
%         Heli.w_dot = 0;
%         Heli.p = 0;
%         Heli.q = 0;
%         Heli.r = 0;
%         Heli.p_dot = 0;
%         Heli.q_dot = 0;
%         Heli.r_dot = 0;
%         
%         InitialStates = [10,10,deg2rad(10),0,0,0,deg2rad(0),deg2rad(0)] ; %v_01,v_02，theta_0,theta_diff,theta_1c,theta_1s,theta_PR,phi
%         options=optimset('Display','off','TolFun',1e-8,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
%         [x,fval,exitflag,~] = fsolve(@AerodynamicsV2,InitialStates,options);
%         
%         % 判断方程是否有解
%         if exitflag>0
%             CalculatePowerV2() 
%             PowerList(itheta) = Heli.Power;
%         else
%             PowerList(itheta) = inf;
%         end
%         
%         itheta = itheta + 1 ;
%     end
%     
%     PowerMin(iV) = min(PowerList);
%     thetaBest(iV) = thetaList(PowerList == min(PowerList));
%     
%     disp(['for V=' num2str(V) ', the best theta(deg) is ' num2str(rad2deg(thetaBest(iV))) ', for a total power of ' num2str(PowerMin(iV)/1000) ' kW.'])
% 
%     iV = iV+1;
% end
% 
% plot(VRange./(Omega*R),rad2deg(thetaBest),'k','linewidth',2)
% xlabel('\mu')
% ylabel('\theta(deg)')
% grid on

%% 1.19 AerodynamicsV3 基础配平测试，无舵面，有尾推，平直平飞/悬停，诱导速度相互干扰并考虑一阶谐波，部件无干扰，半差动。结果不好，自身诱导速度出现负数
% 尾推采用曹燕论文拟合数据
% 俯仰角为冗余变量，需要预设，并且会对应同一配平状态的不同功率
Heli.psi = deg2rad(0); 
% 冗余变量
Heli.theta = deg2rad(4); %预设值

% 运动状态
Heli.V = 0; %悬停,或平直飞行
Heli.u_dot = 0;
Heli.v_dot = 0;
Heli.w_dot = 0;
Heli.p = 0;
Heli.q = 0;
Heli.r = 0;
Heli.p_dot = 0;
Heli.q_dot = 0;
Heli.r_dot = 0;

InitialStates = [10,10,deg2rad(10),0,0,0,deg2rad(0),deg2rad(0)] ; %v_01,v_02，theta_0,theta_diff,theta_1c,theta_1s,theta_PR,phi
options=optimset('Display','iter','TolFun',1e-8,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
[x,fval,exitflag,~] = fsolve(@AerodynamicsV3,InitialStates,options);
if exitflag > 0
   CalculatePowerV2() 
   varNames = {'v_01','v_02','\theta_0(deg)','theta_diff(deg)','theta_1c(deg)','theta_1s(deg)','theta_PR(deg)','Propeller Force(N)','phi(deg)','theta(deg)'};
   varTypes = {'double','double','double','double','double','double','double','double','double','double'};
   Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
   Tvar(1,:) = {Rotor.v_01 Rotor.v_02 rad2deg(Control.theta_0) rad2deg(Control.theta_diff) rad2deg(Control.theta_1c1) rad2deg(Control.theta_1s1) rad2deg(Control.theta_0PR) Propeller.X_PR rad2deg(Heli.phi) rad2deg(Heli.theta) };
   disp(['V: ' num2str(Heli.V)])
   disp(Tvar)
   
   varNames = {'Power1(kW)','Power2(kW)','PowerPR(kW)','PowerP(kW)','TotalPower(kW)'};
   varTypes = {'double','double','double','double','double'};
   Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
   Tvar(1,:) = {Rotor.Power1/1000 Rotor.Power2/1000 Propeller.Power/1000 Heli.PowerP/1000 Heli.Power/1000};
   disp(Tvar)
end

%% 1.19 AerodynamicsV31 基础配平测试，无舵面，有尾推，平直平飞/悬停，诱导速度相互干扰，无一阶谐波，部件无干扰，半差动 ,不太行
% 尾推采用曹燕论文拟合数据
% 俯仰角为冗余变量，需要预设，并且会对应同一配平状态的不同功率
Heli.psi = deg2rad(0); 
% 冗余变量
Heli.theta = deg2rad(4); %预设值

% 运动状态
Heli.V = 0; %悬停,或平直飞行
Heli.u_dot = 0;
Heli.v_dot = 0;
Heli.w_dot = 0;
Heli.p = 0;
Heli.q = 0;
Heli.r = 0;
Heli.p_dot = 0;
Heli.q_dot = 0;
Heli.r_dot = 0;

InitialStates = [10,10,deg2rad(10),0,0,0,deg2rad(0),deg2rad(0)] ; %v_01,v_02，theta_0,theta_diff,theta_1c,theta_1s,theta_PR,phi
options=optimset('Display','iter','TolFun',1e-8,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
[x,~,exitflag,~] = fsolve(@AerodynamicsV31,InitialStates,options);
if exitflag > 0
   CalculatePowerV2() 
   varNames = {'v_01','v_02','theta_0(deg)','theta_diff(deg)','theta_1c(deg)','theta_1s(deg)','theta_PR(deg)','Propeller Force(N)','phi(deg)','theta(deg)'};
   varTypes = {'double','double','double','double','double','double','double','double','double','double'};
   Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
   Tvar(1,:) = {Rotor.v_01 Rotor.v_02 rad2deg(Control.theta_0) rad2deg(Control.theta_diff) rad2deg(Control.theta_1c1) rad2deg(Control.theta_1s1) rad2deg(Control.theta_0PR) Propeller.X_PR rad2deg(Heli.phi) rad2deg(Heli.theta) };
   disp("This is the test for AerodynamicsV31.")
   disp(['V: ' num2str(Heli.V)])
   disp(Tvar)
   
   varNames = {'Power1(kW)','Power2(kW)','PowerPR(kW)','PowerP(kW)','TotalPower(kW)'};
   varTypes = {'double','double','double','double','double'};
   Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
   Tvar(1,:) = {Rotor.Power1/1000 Rotor.Power2/1000 Propeller.Power/1000 Heli.PowerP/1000 Heli.Power/1000};
   disp(Tvar)
end

%% 1.20 AerodynamicsV311 基础配平测试，无舵面，有尾推，平直平飞/悬停，诱导速度相互干扰，无一阶谐波，旋翼对尾翼干扰，尾翼naca0012，半差动 。效果不好。
% Heli.psi = deg2rad(0); 
% % 冗余变量
% Heli.theta = deg2rad(3); %预设值
% 
% % 运动状态
% Heli.V = 0; %悬停,或平直飞行
% Heli.u_dot = 0;
% Heli.v_dot = 0;
% Heli.w_dot = 0;
% Heli.p = 0;
% Heli.q = 0;
% Heli.r = 0;
% Heli.p_dot = 0;
% Heli.q_dot = 0;
% Heli.r_dot = 0;
% 
% InitialStates = [10,10,deg2rad(10),0,0,0,deg2rad(0),deg2rad(0)] ; %v_01,v_02，theta_0,theta_diff,theta_1c,theta_1s,theta_PR,phi
% options=optimset('Display','iter','TolFun',1e-8,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
% [x,fval,exitflag,~] = fsolve(@AerodynamicsV311,InitialStates,options);
% 
% % if solved, calculate power and display infomation
% if exitflag > 0
%    CalculatePowerV2() 
%    varNames = {'v_01','v_02','theta_0(deg)','theta_diff(deg)','theta_1c(deg)','theta_1s(deg)','theta_PR(deg)','Propeller Force(N)','phi(deg)','theta(deg)'};
%    varTypes = {'double','double','double','double','double','double','double','double','double','double'};
%    Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
%    Tvar(1,:) = {Rotor.v_01 Rotor.v_02 rad2deg(Control.theta_0) rad2deg(Control.theta_diff) rad2deg(Control.theta_1c1) rad2deg(Control.theta_1s1) rad2deg(Control.theta_0PR) Propeller.X_PR rad2deg(Heli.phi) rad2deg(Heli.theta) };
%    disp("This is the test for AerodynamicsV311.")
%    disp(['V: ', num2str(Heli.V)])
%    disp(Tvar)
%    
%    varNames = {'Power1(kW)','Power2(kW)','PowerPR(kW)','PowerP(kW)','TotalPower(kW)'};
%    varTypes = {'double','double','double','double','double'};
%    Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
%    Tvar(1,:) = {Rotor.Power1/1000 Rotor.Power2/1000 Propeller.Power/1000 Heli.PowerP/1000 Heli.Power/1000};
%    disp(Tvar)
% end

%% 1.20 AerodynamicsV312 基础配平测试，无舵面，有尾推，平直平飞/悬停，诱导速度相互干扰，无一阶谐波，旋翼对尾翼无干扰，对机身有干扰。半差动 。效果不好。无解。
% Heli.psi = deg2rad(0); 
% % 冗余变量
% Heli.theta = deg2rad(3); %预设值
% 
% % 运动状态
% Heli.V = 0; %悬停,或平直飞行
% Heli.u_dot = 0;
% Heli.v_dot = 0;
% Heli.w_dot = 0;
% Heli.p = 0;
% Heli.q = 0;
% Heli.r = 0;
% Heli.p_dot = 0;
% Heli.q_dot = 0;
% Heli.r_dot = 0;
% 
% InitialStates = [10,10,deg2rad(10),0,0,0,deg2rad(0),deg2rad(0)] ; %v_01,v_02，theta_0,theta_diff,theta_1c,theta_1s,theta_PR,phi
% options=optimset('Display','iter','TolFun',1e-8,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
% [x,fval,exitflag,~] = fsolve(@AerodynamicsV312,InitialStates,options);
% 
% % if solved, calculate power and display infomation
% if exitflag > 0
%    CalculatePowerV2() 
%    varNames = {'v_01','v_02','theta_0(deg)','theta_diff(deg)','theta_1c(deg)','theta_1s(deg)','theta_PR(deg)','Propeller Force(N)','phi(deg)','theta(deg)'};
%    varTypes = {'double','double','double','double','double','double','double','double','double','double'};
%    Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
%    Tvar(1,:) = {Rotor.v_01 Rotor.v_02 rad2deg(Control.theta_0) rad2deg(Control.theta_diff) rad2deg(Control.theta_1c1) rad2deg(Control.theta_1s1) rad2deg(Control.theta_0PR) Propeller.X_PR rad2deg(Heli.phi) rad2deg(Heli.theta) };
%    disp("This is the test for AerodynamicsV311.")
%    disp(['V: ', num2str(Heli.V)])
%    disp(Tvar)
%    
%    varNames = {'Power1(kW)','Power2(kW)','PowerPR(kW)','PowerP(kW)','TotalPower(kW)'};
%    varTypes = {'double','double','double','double','double'};
%    Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
%    Tvar(1,:) = {Rotor.Power1/1000 Rotor.Power2/1000 Propeller.Power/1000 Heli.PowerP/1000 Heli.Power/1000};
%    disp(Tvar)
% end

%% 1.20 AerodynamicsV32 基础配平测试，无舵面，有尾推，平直平飞/悬停，诱导速度相互干扰，无一阶谐波，部件无干扰，半差动。机身尾翼迎角公式有改动。
% 尾推采用曹燕论文拟合数据
% 俯仰角为冗余变量，需要预设，并且会对应同一配平状态的不同功率
Heli.psi = deg2rad(0); 
% 冗余变量
Heli.theta = deg2rad(5); %预设值

% 运动状态
Heli.V = 0; %悬停,或平直飞行
Heli.u_dot = 0;
Heli.v_dot = 0;
Heli.w_dot = 0;
Heli.p = 0;
Heli.q = 0;
Heli.r = 0;
Heli.p_dot = 0;
Heli.q_dot = 0;
Heli.r_dot = 0;

InitialStates = [10,10,deg2rad(10),0,0,0,deg2rad(0),deg2rad(0)] ; %v_01,v_02，theta_0,theta_diff,theta_1c,theta_1s,theta_PR,phi
options=optimset('Display','iter','TolFun',1e-8,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
[x,~,exitflag,~] = fsolve(@AerodynamicsV32,InitialStates,options);
if exitflag > 0
   CalculatePowerV2() 
   varNames = {'v_01','v_02','theta_0(deg)','theta_diff(deg)','theta_1c(deg)','theta_1s(deg)','theta_PR(deg)','Propeller Force(N)','phi(deg)','theta(deg)'};
   varTypes = {'double','double','double','double','double','double','double','double','double','double'};
   Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
   Tvar(1,:) = {Rotor.v_01 Rotor.v_02 rad2deg(Control.theta_0) rad2deg(Control.theta_diff) rad2deg(Control.theta_1c1) rad2deg(Control.theta_1s1) rad2deg(Control.theta_0PR) Propeller.X_PR rad2deg(Heli.phi) rad2deg(Heli.theta) };
   disp("################################This is the test for AerodynamicsV32.")
   disp(['V: ' num2str(Heli.V)])
   disp(Tvar)
   
   varNames = {'Power1(kW)','Power2(kW)','PowerPR(kW)','PowerP(kW)','TotalPower(kW)'};
   varTypes = {'double','double','double','double','double'};
   Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
   Tvar(1,:) = {Rotor.Power1/1000 Rotor.Power2/1000 Propeller.Power/1000 Heli.PowerP/1000 Heli.Power/1000};
   disp(Tvar)
end

%% 1.20 AerodynamicsV321 基础配平测试，无舵面，有尾推，平直平飞/悬停，诱导速度相互干扰，无一阶谐波，部件无干扰，半差动。机身尾翼迎角公式有改动。尾翼使用naca0012。(5.7,0.008)
% 尾推采用曹燕论文拟合数据
% 俯仰角为冗余变量，需要预设，并且会对应同一配平状态的不同功率
Heli.psi = deg2rad(0); 
% 冗余变量
Heli.theta = deg2rad(3); %预设值

% 运动状态
Heli.V = 100; %悬停,或平直飞行
Heli.u_dot = 0;
Heli.v_dot = 0;
Heli.w_dot = 0;
Heli.p = 0;
Heli.q = 0;
Heli.r = 0;
Heli.p_dot = 0;
Heli.q_dot = 0;
Heli.r_dot = 0;

InitialStates = [10,10,deg2rad(10),0,0,0,deg2rad(0),deg2rad(0)] ; %v_01,v_02，theta_0,theta_diff,theta_1c,theta_1s,theta_PR,phi
options=optimset('Display','iter','TolFun',1e-8,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
[x,~,exitflag,~] = fsolve(@AerodynamicsV321,InitialStates,options);
if exitflag > 0
   CalculatePowerV2() 
   varNames = {'v_01','v_02','theta_0(deg)','theta_diff(deg)','theta_1c(deg)','theta_1s(deg)','theta_PR(deg)','Propeller Force(N)','phi(deg)','theta(deg)'};
   varTypes = {'double','double','double','double','double','double','double','double','double','double'};
   Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
   Tvar(1,:) = {Rotor.v_01 Rotor.v_02 rad2deg(Control.theta_0) rad2deg(Control.theta_diff) rad2deg(Control.theta_1c1) rad2deg(Control.theta_1s1) rad2deg(Control.theta_0PR) Propeller.X_PR rad2deg(Heli.phi) rad2deg(Heli.theta) };
   disp("################################This is the test for AerodynamicsV321.")
   disp(['V: ' num2str(Heli.V)])
   disp(Tvar)
   
   varNames = {'Power1(kW)','Power2(kW)','PowerPR(kW)','PowerP(kW)','TotalPower(kW)'};
   varTypes = {'double','double','double','double','double'};
   Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
   Tvar(1,:) = {Rotor.Power1/1000 Rotor.Power2/1000 Propeller.Power/1000 Heli.PowerP/1000 Heli.Power/1000};
   disp(Tvar)
end

%% 1.20 AerodynamicsV4 承接V4（V3版本不行）基础配平测试，无舵面，有尾推，平直平飞/悬停，不含不含部件间气动干扰以及旋翼间干扰，半差动。机身、尾翼迎角公式有改动,naca0012
% 尾推采用曹燕论文拟合数据
% 俯仰角为冗余变量，需要预设，并且会对应同一配平状态的不同功率
Heli.psi = deg2rad(0); 
% 冗余变量
Heli.theta = deg2rad(3); %预设值

% 运动状态
Heli.V = 0; %悬停,或平直飞行
Heli.u_dot = 0;
Heli.v_dot = 0;
Heli.w_dot = 0;
Heli.p = 0;
Heli.q = 0;
Heli.r = 0;
Heli.p_dot = 0;
Heli.q_dot = 0;
Heli.r_dot = 0;

InitialStates = [10,10,deg2rad(10),0,0,0,deg2rad(0),deg2rad(0)] ; %v_01,v_02，theta_0,theta_diff,theta_1c,theta_1s,theta_PR,phi
options=optimset('Display','iter','TolFun',1e-8,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
[x,fval,exitflag,~] = fsolve(@AerodynamicsV4,InitialStates,options);
if exitflag > 0
   CalculatePowerV2() 
   varNames = {'v_01','v_02','theta_0(deg)','theta_diff(deg)','theta_1c(deg)','theta_1s(deg)','theta_PR(deg)','Propeller Force(N)','phi(deg)','theta(deg)'};
   varTypes = {'double','double','double','double','double','double','double','double','double','double'};
   Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
   Tvar(1,:) = {Rotor.v_01 Rotor.v_02 rad2deg(Control.theta_0) rad2deg(Control.theta_diff) rad2deg(Control.theta_1c1) rad2deg(Control.theta_1s1) rad2deg(Control.theta_0PR) Propeller.X_PR rad2deg(Heli.phi) rad2deg(Heli.theta) };
   disp("################################ This is the test for AerodynamicsV4.")
   disp(['V: ' num2str(Heli.V)])
   disp(Tvar)
   
   varNames = {'Power1(kW)','Power2(kW)','PowerPR(kW)','PowerP(kW)','TotalPower(kW)'};
   varTypes = {'double','double','double','double','double'};
   Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
   Tvar(1,:) = {Rotor.Power1/1000 Rotor.Power2/1000 Propeller.Power/1000 Heli.PowerP/1000 Heli.Power/1000};
   disp(Tvar)
end

%% 1.22 AerodynamicsV5 在V4基础上加入操纵, 4个冗余操纵：delta_e,delta_r, theta_1c_diff,theta_1s_diff, 以及theta俯仰角
Heli.psi = deg2rad(0); 
% 俯仰角为冗余变量，需要预设，并且会对应同一配平状态的不同功率
% 冗余变量
Heli.theta = deg2rad(3); %预设值
Control.delta_e = deg2rad(0);
Control.delta_r = deg2rad(0);
Control.theta_1c_diff = deg2rad(0);
Control.theta_1s_diff = deg2rad(0);

% 运动状态
Heli.V = 0; %悬停,或平直飞行
Heli.u_dot = 0;
Heli.v_dot = 0;
Heli.w_dot = 0;
Heli.p = 0;
Heli.q = 0;
Heli.r = 0;
Heli.p_dot = 0;
Heli.q_dot = 0;
Heli.r_dot = 0;

InitialStates = [10,10,deg2rad(10),0,0,0,deg2rad(0),deg2rad(0)] ; %v_01,v_02，theta_0,theta_diff,theta_1c,theta_1s,theta_PR,phi
options=optimset('Display','iter','TolFun',1e-8,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
[x,fval,exitflag,~] = fsolve(@AerodynamicsV5,InitialStates,options);
if exitflag > 0
   CalculatePowerV2() 
   varNames = {'v_01','v_02','theta_0(deg)','theta_diff(deg)','theta_1c(deg)','theta_1s(deg)','theta_PR(deg)','Propeller Force(N)','phi(deg)','theta(deg)'};
   varTypes = {'double','double','double','double','double','double','double','double','double','double'};
   Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
   Tvar(1,:) = {Rotor.v_01 Rotor.v_02 rad2deg(Control.theta_0) rad2deg(Control.theta_diff) rad2deg(Control.theta_1c1) rad2deg(Control.theta_1s1) rad2deg(Control.theta_0PR) Propeller.X_PR rad2deg(Heli.phi) rad2deg(Heli.theta) };
   disp("################################ This is the test for AerodynamicsV5.")
   disp(['V: ' num2str(Heli.V)])
   disp(Tvar)
   
   varNames = {'Power1(kW)','Power2(kW)','PowerPR(kW)','PowerP(kW)','TotalPower(kW)'};
   varTypes = {'double','double','double','double','double'};
   Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
   Tvar(1,:) = {Rotor.Power1/1000 Rotor.Power2/1000 Propeller.Power/1000 Heli.PowerP/1000 Heli.Power/1000};
   disp(Tvar)
end