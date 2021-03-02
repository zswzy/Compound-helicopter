%% Main file
clear all
clc
%% 定义基本参数
% 飞行高度(海平面）
global h
h = 1000;

% 大气数据
global rho
[~,~,~,rho] = atmosisa(h); %温度，音速，压强，密度

global g
% 重力加速度
g = 9.81;

global N_b m_b z_diff R A Omega sigma_M theta_t omega_n I_beta a c_d c K_beta gamma_M epsilon e M_beta i_theta i_phi x_H1 y_H1 z_H1 x_H2 y_H2 z_H2
% 上下旋翼各自桨叶数
N_b = 3;
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
sigma_M = 0.127;
% 主旋翼桨叶扭转角 rad
theta_t = deg2rad(-10);
% 主旋翼一阶挥舞固有频率 
omega_n = 1.4*Omega;
% 主旋翼挥舞惯性矩 kg m^2
I_beta = 450;
% 主旋翼升力线斜率,NACA0012
a = 5.7;
%主旋翼桨叶阻力系数
c_d = 0.008;
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
e = 2.58;
epsilon = e/R;
% 主旋翼对挥舞铰的质量静矩 200多
% M_beta = 1/2*R^2*m_b/R; 
% M_beta = I_beta*3/2*(1/R)/(1-epsilon);
M_beta = 123;
% 主旋翼桨毂纵向安装角
i_theta = deg2rad(3);
% 主旋翼桨毂横向安装角
i_phi = 0;
% 主旋翼位置 1：下旋翼，2：上旋翼
x_H1 = 0;
y_H1 = 0;
z_H1 = -0.89;
x_H2 = 0;
y_H2 = 0;
z_H2 = z_H1-z_diff;


global R_PR Omega_PR sigma_PR theta_tPR  a_PR c_dPR x_PR y_PR z_PR K_PR I_beta_PR c_PR kappa
% 推进桨半径 m
R_PR = 1.3;
% 推进桨转速 rad/s 162
Omega_PR = 162;
% 推进桨实度
sigma_PR = 0.2;
% 推进桨叶扭转
theta_tPR = deg2rad(-30);
% 推进桨叶升力线斜率,NACA0012
a_PR = 5.7;
% 推进桨叶阻力系数
c_dPR = 0.008;
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

global x_F y_F z_F l_F s_F 
% 机身气动中心
x_F = 0;
y_F = 0;
z_F = 0;
% 机身长度
l_F = 12;
% 机身最大迎风面积
s_F = 2.4*10;

global alpha_0HS s_H s_e xi_H x_HS y_HS z_HS
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

global alpha_0VS x_VS y_VS z_VS s_r xi_V s_V
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

global L_H1B1 L_H2B2
% 转换矩阵
L_H1B1 = [cos(i_theta)              0           -sin(i_theta);
        sin(i_theta)*sin(i_phi)   cos(i_phi)  cos(i_theta)*sin(i_phi);
        sin(i_theta)*cos(i_phi)   -sin(i_phi) cos(i_theta)*cos(i_phi)];
L_H2B2 = [cos(i_theta)              0           -sin(i_theta);
        sin(i_theta)*sin(i_phi)   cos(i_phi)  cos(i_theta)*sin(i_phi);
        sin(i_theta)*cos(i_phi)   -sin(i_phi) cos(i_theta)*cos(i_phi)];
    
%% 拟合气动参数
% % 插值类型定义
% global fittype
% fittype =  'thinplateinterp';
% % fittype =  'linearinterp';
% % 是否显示拟合结果
% show_figure = true;
% run dataFit.m

% delta_l, delta_u
% Table=readtable("OriginalData.xlsx","Sheet","delta","Range","B1:J3");
% amu      = Table{1,:}';
% adelta_l = Table{2,:}';
% adelta_u = Table{3,:}';
% delta_lFitted = fit(amu, adelta_l, 'linearinterp');
% delta_uFitted = fit(amu, adelta_u, 'linearinterp');
% 
% if show_figure == true
%     amu = 0:0.01:0.5;
%     [adelta_l,adelta_u] = arrayfun(@(x) CalculateDelta_lu(x,delta_lFitted,delta_uFitted) ,amu);
%     plot(amu,adelta_l,'k',amu,adelta_u,'r')
%     hold on
%     title('\delta_l,\delta_u')
%     xlabel('\mu')
%     ylabel('\delta_l,\delta_u')
%     legend('\delta_l','\delta_u')
%     grid on
% end

global delta_lFitted delta_uFitted
Table=readtable("OriginalData.xlsx","Sheet","delta","Range","B1:J3");
amu      = Table{1,:}';
adelta_l = Table{2,:}';
adelta_u = Table{3,:}';
delta_lFitted = fit(amu, adelta_l, 'linearinterp');
delta_uFitted = fit(amu, adelta_u, 'linearinterp');


% X_PR, L_PR
global X_PRFitted L_PRFitted
Table=readtable("OriginalData.xlsx","Sheet","propeller","Range","B1:J3");
atheta_0PR = deg2rad(Table{1,:}');
aX_PR = Table{2,:}';
aL_PR = Table{3,:}';
X_PRFitted = fit(atheta_0PR, aX_PR, 'linearinterp');
L_PRFitted = fit(atheta_0PR, aL_PR, 'linearinterp');
   
%% 旋翼气动力
% 若已知u v w p q r
% 计算前进比mu
% run CalculateMu.m
% % 计算delta
% run CalculateDelta_lu.m
% % 计算入流lambda
% run CalculateLambda.m
% 
% % 计算beta
% beta_S1 = asin(v_H1H1/sqrt(u_H1H1^2+v_H1H1^2));
% beta_S2 = asin(v_H2H2/sqrt(u_H2H2^2+v_H2H2^2));
% % 计算转换矩阵
% L_S1H1 = [cos(beta_S1)     sin(beta_S1) 0;
%          -sin(beta_S1)    cos(beta_S1)  0;
%             0               0           1];
% L_S2H2 = [cos(beta_S2)     sin(beta_S2) 0;
%          -sin(beta_S2)    cos(beta_S2)  0;
%             0               0           1];
% % 计算p_S1 q_S1 r_S1
% run CalculatePqrS.m
% 
% % 计算旋翼挥舞
% run CalculateFlapping.m
% 
% % 计算旋翼的力
% run CalculateRotorForceMoment.m
% 
% % 用诱导速度计算旋翼拉力 I代表诱导速度
% T_W1_I = 2*rho*A*v_1*sqrt(u_H1H1^2+v_H1H1^2+(-w_H1H1+v_1+delta_l*v_2)^2);
% T_W2_I = 2*rho*A*v_2*sqrt(u_H2H2^2+v_H2H2^2+(-w_H2H2+v_2+delta_2*v_1)^2);
% 
% % 计算 机身受力
% run CalculateFuselageForceMoment.m %OK

%% 配平
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%配平状态
global V psi p q r 
h = 1000;
V = 0.000001;
%预设值
psi=            0;   
p=              0;
q=              0;
r=              0;
global theta A_diff B_diff delta_e delta_r
% 冗余变量
theta = deg2rad(3);
A_diff = 0;
B_diff = 0;
delta_e = 0;
delta_r = 0;
%
S.theta_0 = deg2rad(20);% States(1): theta_0
S.theta_diff =deg2rad(-1);% States(2): theta_diff
S.A =0;% States(3): A
S.B =0;% States(4): B
S.theta_PR =deg2rad(30);% States(5): theta_PR
S.v_0PR =5;% States(6): v0_PR
S.v_1 =10;% States(7): v_1
S.v_2 =10;% States(8): v_2
S.a_01 =0;% States(9): a_01
S.a_11 =0;% States(10): a_11
S.b_11 =0;% States(11): b_11
S.a_02 =0;% States(12): a_02
S.a_12 =0;% States(13): a_12
S.b_12 =0;% States(14): b_12
S.phi =deg2rad(6);% States(15): phi

% 计算测试
%InitialStates = [S.theta_0 S.theta_diff S.A S.B S.theta_PR S.v_0PR S.v_1 S.v_2 S.a_01 S.a_11 S.b_11 S.a_02 S.a_12 S.b_12 S.phi];
%F = AllAerodynamics(InitialStates);
%
options=optimset('Display','iter','TolFun',1e-18,'Maxiter',5000,'Algorithm','trust-region-dogleg' ,'MaxFunEvals',20000);
InitialStates = [S.theta_0 S.theta_diff S.A S.B S.theta_PR S.v_0PR S.v_1 S.v_2 S.a_01 S.a_11 S.b_11 S.a_02 S.a_12 S.b_12 S.phi];

%F = AllAerodynamics(InitialStates);
global display
display = false;
[x,fval,exitflag,~] = fsolve(@AllAerodynamics,InitialStates,options);
varNames = {'theta_0(deg)','theta_diff(deg)','A','B','theta_PR(deg)','v_0PR','v_1','v_2','a_01','a_11','b_11','a_02','a_12','b_12','phi(deg)'};
varTypes = {'double','double','double','double','double','double','double','double','double','double','double','double','double','double','double'};
Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
Tvar(1,:) = {rad2deg(x(1)) rad2deg(x(2)) x(3) x(4) rad2deg(x(5)) x(6) x(7) x(8) x(9) x(10) x(11) x(12) x(13) x(14) rad2deg(x(15))};
disp(['V: ' num2str(V)])
disp(['theta(deg): ' num2str(rad2deg(theta))])
disp(Tvar)
if exitflag > 0
    display = true;
    F = AllAerodynamics(x);
end

S.theta_0 = x(1);
S.theta_diff =x(2);
S.A =x(3);
S.B =x(4);
S.theta_PR =x(5);
S.v_0PR =x(6);
S.v_1 =x(7);
S.v_2 =x(8);
S.a_01 =x(9);
S.a_11 =x(10);
S.b_11 =x(11);
S.a_02 =x(12);
S.a_12 =x(13);
S.b_12 =x(14);
S.phi =x(15);

%% 改变theta， 找到悬停状态下的最小功率
global display
display = false;
aPW_MR1 = [];aPW_MR2 = [];aPW_PR = [];aPW_Total = [];

% theta 寻找范围
theta_range = [];
%for theta  = [deg2rad(2.5) deg2rad(4)]
for theta  = linspace(deg2rad(-5),deg2rad(5),100)
    % 给定theta，先配平
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %配平状态
    global V psi p q r
    h = 1000;
    V = 0.0000000001;
    %预设值
    psi=            0;
    p=              0;
    q=              0;
    r=              0;
    global theta A_diff B_diff delta_e delta_r
    % 冗余变量
    %theta = deg2rad(2.5);
    A_diff = 0;
    B_diff = 0;
    delta_e = 0;
    delta_r = 0;
    %
    S.theta_0 = deg2rad(20);% States(1): theta_0
    S.theta_diff =deg2rad(-1);% States(2): theta_diff
    S.A =0;% States(3): A
    S.B =0;% States(4): B
    S.theta_PR =deg2rad(30);% States(5): theta_PR
    S.v_0PR =5;% States(6): v0_PR
    S.v_1 =10;% States(7): v_1
    S.v_2 =10;% States(8): v_2
    S.a_01 =0;% States(9): a_01
    S.a_11 =0;% States(10): a_11
    S.b_11 =0;% States(11): b_11
    S.a_02 =0;% States(12): a_02
    S.a_12 =0;% States(13): a_12
    S.b_12 =0;% States(14): b_12
    S.phi =deg2rad(6);% States(15): phi
    
    options=optimset('Display','off','TolFun',1e-18,'Maxiter',5000,'Algorithm','trust-region-dogleg' ,'MaxFunEvals',20000);
    InitialStates = [S.theta_0 S.theta_diff S.A S.B S.theta_PR S.v_0PR S.v_1 S.v_2 S.a_01 S.a_11 S.b_11 S.a_02 S.a_12 S.b_12 S.phi];
    
    %F = AllAerodynamics(InitialStates);
    
    [x,fval,exitflag,~] = fsolve(@AllAerodynamics,InitialStates,options);
%    varNames = {'theta_0(deg)','theta_diff(deg)','A','B','theta_PR(deg)','v_0PR','v_1','v_2','a_01','a_11','b_11','a_02','a_12','b_12','phi(deg)'};
%    varTypes = {'double','double','double','double','double','double','double','double','double','double','double','double','double','double','double'};
%    Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
%    Tvar(1,:) = {rad2deg(x(1)) rad2deg(x(2)) x(3) x(4) rad2deg(x(5)) x(6) x(7) x(8) x(9) x(10) x(11) x(12) x(13) x(14) rad2deg(x(15))};
%     disp(['V: ' num2str(V)])
%     disp(['theta(deg): ' num2str(rad2deg(theta))])
%     disp(Tvar)
    
States = zeros(1,15);
    if exitflag > 0
        States(1) = x(1);
        States(2) =x(2);
        States(3) =x(3);
        States(4) =x(4);
        States(5) =x(5);
        States(6) =x(6);
        States(7) =x(7);
        States(8) =x(8);
        States(9) =x(9);
        States(10) =x(10);
        States(11) =x(11);
        States(12) =x(12);
        States(13) =x(13);
        States(14) =x(14);
        States(15) =x(15);
        [PW_MR1,PW_MR2,PW_PR,PW_Total] = CalculatePower(States);
        aPW_MR1 = [aPW_MR1 PW_MR1];
        aPW_MR2 = [aPW_MR2 PW_MR2];
        aPW_PR = [aPW_PR PW_PR];
        aPW_Total = [aPW_Total PW_Total];
        theta_range = [theta_range theta];
    end
end

%变成kW，deg
aPW_MR1 = aPW_MR1/1000;
aPW_MR2 = aPW_MR2/1000;
aPW_PR = aPW_PR/1000;
aPW_Total = aPW_Total/1000;
theta_range = rad2deg(theta_range);

%画图
figure(1)
plot(theta_range,aPW_MR1,theta_range,aPW_MR2,theta_range,aPW_PR,theta_range,aPW_Total,'linewidth',1.5)
legend('Lower rotor','Upper rotor','Propeller','Tatal power')
xlabel('\theta (deg)')
ylabel('Power (kW)')
title('Relation of required power and pitch angle \theta in hover state')

figure(2)
plot(theta_range,aPW_Total,'linewidth',1.5)
legend('Tatal power')
xlabel('\theta (deg)')
ylabel('Power (kW)')
title('Relation of required power and pitch angle \theta in hover state')
                                                        