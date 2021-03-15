function function_trim_no_redundant()
% 2.19 整机配平 无冗余变量
% 生成全包线飞行配平数据 trim_result_no_redundant.csv 并画图
h = 100;
rho = 1.2133;

%% 建立对象
% 建立对象

% 直升机
DoubleRotorHelicopter       = Helicopter();
DoubleRotorHelicopter.GW    = 5500;
DoubleRotorHelicopter.GWF   = 5500*9.81;

% 下旋翼
LowerRotor = RotorFixed('anticlockwise'); % 下旋翼逆时针转， 为默认情况
LowerRotor.a_0          = 7.6;           % 主旋翼升力线斜率,NACA0012
LowerRotor.b            = 3;             % 旋翼桨叶数
LowerRotor.c            = 0.29;          % 主旋翼桨叶弦长m
LowerRotor.delta        = 0.01;         % 主旋翼桨叶阻力系数
LowerRotor.e            = 2.58/5.49;     % 无量纲等效铰偏置量
LowerRotor.e_oswald     = 0.7;           % 奥斯瓦尔德效率因子 (0.8)
LowerRotor.gamma_s      = deg2rad(3);    % 主旋翼桨毂纵向安装角
LowerRotor.h_R          = 0.89;          % 主旋翼处于重心之上的位置
LowerRotor.m_b          = 60;            % 桨叶质量
LowerRotor.omega_n      = 1.4*35.9;      % 主旋翼一阶挥舞固有频率 
LowerRotor.rho          = rho;           % 空气密度
LowerRotor.s            = 0.127;         % 主旋翼实度
LowerRotor.theta_t      = deg2rad(-10);  % 主旋翼桨叶扭转角 rad
LowerRotor.x_cg         = 0;             % 主旋翼处于重心之后的位置
LowerRotor.x_H          = 0;             % 桨毂x位置
LowerRotor.y_H          = 0;             % 桨毂y位置
LowerRotor.z_H          = -0.89;         % 桨毂z位置
LowerRotor.z_diff       = 0.77;          % 上下旋翼间距 m
LowerRotor.I_beta       = 450;           % 主旋翼挥舞惯性矩 kg m^2
LowerRotor.K_beta       = 220500;        % K_beta 弹簧刚度  Nm/rad
LowerRotor.M_beta       = 123;           % 主旋翼对挥舞铰的质量静矩
LowerRotor.Omega        = 35.9;          % 主旋翼转速rad/s
LowerRotor.R            = 5.49;          % 主旋翼半径 m

% 上旋翼
UpperRotor = RotorFixed('clockwise'); % 上旋翼顺时针转
UpperRotor.a_0          = 7.6;           % 主旋翼升力线斜率,NACA0012
UpperRotor.b            = 3;             % 旋翼桨叶数
UpperRotor.c            = 0.29;          % 主旋翼桨叶弦长m
UpperRotor.delta        = 0.01;         % 主旋翼桨叶阻力系数
UpperRotor.e            = 2.58/5.49;     % 无量纲等效铰偏置量
UpperRotor.e_oswald     = 0.7;           % 奥斯瓦尔德效率因子 (0.8)
UpperRotor.gamma_s      = deg2rad(3);    % 主旋翼桨毂纵向安装角
UpperRotor.h_R          = 0.89+0.77;     % 主旋翼处于重心之上的位置
UpperRotor.m_b          = 60;            % 桨叶质量
UpperRotor.omega_n      = 1.4*35.9;      % 主旋翼一阶挥舞固有频率 
UpperRotor.rho          = rho;           % 空气密度
UpperRotor.s            = 0.127;         % 主旋翼实度
UpperRotor.theta_t      = deg2rad(-10);  % 主旋翼桨叶扭转角 rad
UpperRotor.x_cg         = 0;             % 主旋翼处于重心之后的位置
UpperRotor.x_H          = 0;             % 桨毂x位置
UpperRotor.y_H          = 0;             % 桨毂y位置
UpperRotor.z_H          = -0.89-0.77;    % 桨毂z位置
UpperRotor.z_diff       = 0.77;          % 上下旋翼间距 m
UpperRotor.I_beta       = 450;           % 主旋翼挥舞惯性矩 kg m^2
UpperRotor.K_beta       = 220500;        % K_beta 弹簧刚度  Nm/rad
UpperRotor.M_beta       = 123;           % 主旋翼对挥舞铰的质量静矩
UpperRotor.Omega        = 35.9;          % 主旋翼转速rad/s
UpperRotor.R            = 5.49;          % 主旋翼半径 m

% 推进桨
Prop = PropellerNUAA();
Prop.delta              = 0.008;
Prop.e_oswald           = 0.8;
Prop.rho                = rho;
Prop.s                  = 0.2;
Prop.x_PR                = -7.66;
Prop.y_PR                = 0;
Prop.z_PR                = 0;
Prop.Omega              = 162;
Prop.R                  = 1.3;

% 机身
Fus = Fuselage();
Fus.rho                 = rho;
Fus.R                   = 5.49;
Fus.A                   = pi*5.49^2;
Fus.s_F                 = (5.6+5.6)*0.1+pi*1.5^2;
Fus.x_F                 = 0;
Fus.y_F                 = 0;
Fus.z_F                 = 0;

% 平尾
HorStab = HorizontalStabilizerSimple();
HorStab.a_0             = 3.2;
HorStab.delta           = 0.001;
HorStab.rho             = rho;
HorStab.s_HS            = 5.6;
HorStab.s_e             = 5.6*0.1;
HorStab.xi              = 0;
HorStab.K_HS            = 0.8;
HorStab.x_HS            = -6.8;
HorStab.y_HS            = 0;
HorStab.z_HS            = 0.2;

% 垂尾
VerStab = VerticalStabilizerSimple();
VerStab.a_0             = 2;
VerStab.delta           = 0.001;
VerStab.rho             = rho;
VerStab.s_VS            = 5.6;
VerStab.s_r             = 5.6*0.1;
VerStab.xi              = 0;
VerStab.K_VS            = 0.8;
VerStab.x_VS            = -6.8;
VerStab.y_VS            = 0;
VerStab.z_VS            = -0.5;

% 建立结构体
Rotorcraft.DoubleRotorHelicopter    = DoubleRotorHelicopter;
Rotorcraft.LowerRotor               = LowerRotor;
Rotorcraft.UpperRotor               = UpperRotor;
Rotorcraft.Prop                     = Prop;
Rotorcraft.Fus                      = Fus;
Rotorcraft.HorStab                  = HorStab;  
Rotorcraft.VerStab                  = VerStab;

%% 不同速度下的配平
number_of_U = 101;
array_U = linspace(0,100,number_of_U);
matrix_trim_states = zeros(number_of_U,21);
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
                                    Aerodynamics_trim_full_8var, ... 
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



end

