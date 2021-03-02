% 2.14 fuselage 测试
clearvars
clear global
clc
h = 10;
[~,~,~,rho] = atmosisa(h);

% 直升机
DoubleRotorHelicopter       = Helicopter();
DoubleRotorHelicopter.GW    = 5500;
DoubleRotorHelicopter.GWF   = 5500*9.81;

Fus = Fuselage();

Fus.rho = rho;
Fus.R = 5.49;
Fus.A = pi*5.49^2;
Fus.x_F = 0;
Fus.y_F = 0;
Fus.z_F = 0;


DoubleRotorHelicopter.u         = 0;
DoubleRotorHelicopter.v         = 0;
DoubleRotorHelicopter.w         = 0;
DoubleRotorHelicopter.u_dot     = 0;
DoubleRotorHelicopter.v_dot     = 0;
DoubleRotorHelicopter.w_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;

Fus.u = DoubleRotorHelicopter.u;
Fus.v = DoubleRotorHelicopter.v;
Fus.w = DoubleRotorHelicopter.w;
Fus.u_dot = DoubleRotorHelicopter.u_dot;
Fus.v_dot = DoubleRotorHelicopter.v_dot;
Fus.w_dot = DoubleRotorHelicopter.w_dot;
Fus.p = DoubleRotorHelicopter.p;
Fus.q = DoubleRotorHelicopter.q;
Fus.r = DoubleRotorHelicopter.r;
Fus.p_dot = DoubleRotorHelicopter.p_dot;
Fus.q_dot = DoubleRotorHelicopter.q_dot;
Fus.r_dot = DoubleRotorHelicopter.r_dot;

Fus.calculate_force()
Fus.calculate_torque()

%% 实例1 前飞
DoubleRotorHelicopter.u         = 10;
DoubleRotorHelicopter.v         = 0;
DoubleRotorHelicopter.w         = 0;
DoubleRotorHelicopter.u_dot     = 0;
DoubleRotorHelicopter.v_dot     = 0;
DoubleRotorHelicopter.w_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;

Fus.u = DoubleRotorHelicopter.u;
Fus.v = DoubleRotorHelicopter.v;
Fus.w = DoubleRotorHelicopter.w;
Fus.u_dot = DoubleRotorHelicopter.u_dot;
Fus.v_dot = DoubleRotorHelicopter.v_dot;
Fus.w_dot = DoubleRotorHelicopter.w_dot;
Fus.p = DoubleRotorHelicopter.p;
Fus.q = DoubleRotorHelicopter.q;
Fus.r = DoubleRotorHelicopter.r;
Fus.p_dot = DoubleRotorHelicopter.p_dot;
Fus.q_dot = DoubleRotorHelicopter.q_dot;
Fus.r_dot = DoubleRotorHelicopter.r_dot;

Fus.calculate_force()
Fus.calculate_torque()

%% 实例2 后飞
DoubleRotorHelicopter.u         = -10;
DoubleRotorHelicopter.v         = 0;
DoubleRotorHelicopter.w         = 0;
DoubleRotorHelicopter.u_dot     = 0;
DoubleRotorHelicopter.v_dot     = 0;
DoubleRotorHelicopter.w_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;

Fus.u = DoubleRotorHelicopter.u;
Fus.v = DoubleRotorHelicopter.v;
Fus.w = DoubleRotorHelicopter.w;
Fus.u_dot = DoubleRotorHelicopter.u_dot;
Fus.v_dot = DoubleRotorHelicopter.v_dot;
Fus.w_dot = DoubleRotorHelicopter.w_dot;
Fus.p = DoubleRotorHelicopter.p;
Fus.q = DoubleRotorHelicopter.q;
Fus.r = DoubleRotorHelicopter.r;
Fus.p_dot = DoubleRotorHelicopter.p_dot;
Fus.q_dot = DoubleRotorHelicopter.q_dot;
Fus.r_dot = DoubleRotorHelicopter.r_dot;

Fus.calculate_force()
Fus.calculate_torque()

%% 实例3 右飞
DoubleRotorHelicopter.u         = 0;
DoubleRotorHelicopter.v         = 10;
DoubleRotorHelicopter.w         = 0;
DoubleRotorHelicopter.u_dot     = 0;
DoubleRotorHelicopter.v_dot     = 0;
DoubleRotorHelicopter.w_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;

Fus.u = DoubleRotorHelicopter.u;
Fus.v = DoubleRotorHelicopter.v;
Fus.w = DoubleRotorHelicopter.w;
Fus.u_dot = DoubleRotorHelicopter.u_dot;
Fus.v_dot = DoubleRotorHelicopter.v_dot;
Fus.w_dot = DoubleRotorHelicopter.w_dot;
Fus.p = DoubleRotorHelicopter.p;
Fus.q = DoubleRotorHelicopter.q;
Fus.r = DoubleRotorHelicopter.r;
Fus.p_dot = DoubleRotorHelicopter.p_dot;
Fus.q_dot = DoubleRotorHelicopter.q_dot;
Fus.r_dot = DoubleRotorHelicopter.r_dot;

Fus.calculate_force()
Fus.calculate_torque()

%% 实例4 上飞
DoubleRotorHelicopter.u         = 0;
DoubleRotorHelicopter.v         = 0;
DoubleRotorHelicopter.w         = -10;
DoubleRotorHelicopter.u_dot     = 0;
DoubleRotorHelicopter.v_dot     = 0;
DoubleRotorHelicopter.w_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;

Fus.u = DoubleRotorHelicopter.u;
Fus.v = DoubleRotorHelicopter.v;
Fus.w = DoubleRotorHelicopter.w;
Fus.u_dot = DoubleRotorHelicopter.u_dot;
Fus.v_dot = DoubleRotorHelicopter.v_dot;
Fus.w_dot = DoubleRotorHelicopter.w_dot;
Fus.p = DoubleRotorHelicopter.p;
Fus.q = DoubleRotorHelicopter.q;
Fus.r = DoubleRotorHelicopter.r;
Fus.p_dot = DoubleRotorHelicopter.p_dot;
Fus.q_dot = DoubleRotorHelicopter.q_dot;
Fus.r_dot = DoubleRotorHelicopter.r_dot;

Fus.calculate_force()
Fus.calculate_torque()

%% 实例5 右前飞
DoubleRotorHelicopter.u         = 20;
DoubleRotorHelicopter.v         = 20;
DoubleRotorHelicopter.w         = 0;
DoubleRotorHelicopter.u_dot     = 0;
DoubleRotorHelicopter.v_dot     = 0;
DoubleRotorHelicopter.w_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;

Fus.u = DoubleRotorHelicopter.u;
Fus.v = DoubleRotorHelicopter.v;
Fus.w = DoubleRotorHelicopter.w;
Fus.u_dot = DoubleRotorHelicopter.u_dot;
Fus.v_dot = DoubleRotorHelicopter.v_dot;
Fus.w_dot = DoubleRotorHelicopter.w_dot;
Fus.p = DoubleRotorHelicopter.p;
Fus.q = DoubleRotorHelicopter.q;
Fus.r = DoubleRotorHelicopter.r;
Fus.p_dot = DoubleRotorHelicopter.p_dot;
Fus.q_dot = DoubleRotorHelicopter.q_dot;
Fus.r_dot = DoubleRotorHelicopter.r_dot;

Fus.calculate_force()
Fus.calculate_torque()

%% 实例5 左前飞
DoubleRotorHelicopter.u         = 20;
DoubleRotorHelicopter.v         = -20;
DoubleRotorHelicopter.w         = 0;
DoubleRotorHelicopter.u_dot     = 0;
DoubleRotorHelicopter.v_dot     = 0;
DoubleRotorHelicopter.w_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;

Fus.u = DoubleRotorHelicopter.u;
Fus.v = DoubleRotorHelicopter.v;
Fus.w = DoubleRotorHelicopter.w;
Fus.u_dot = DoubleRotorHelicopter.u_dot;
Fus.v_dot = DoubleRotorHelicopter.v_dot;
Fus.w_dot = DoubleRotorHelicopter.w_dot;
Fus.p = DoubleRotorHelicopter.p;
Fus.q = DoubleRotorHelicopter.q;
Fus.r = DoubleRotorHelicopter.r;
Fus.p_dot = DoubleRotorHelicopter.p_dot;
Fus.q_dot = DoubleRotorHelicopter.q_dot;
Fus.r_dot = DoubleRotorHelicopter.r_dot;

Fus.calculate_force()
Fus.calculate_torque()

%% 实例6 
DoubleRotorHelicopter.u         = 20;
DoubleRotorHelicopter.v         = 6;
DoubleRotorHelicopter.w         = 0;
DoubleRotorHelicopter.u_dot     = 0;
DoubleRotorHelicopter.v_dot     = 0;
DoubleRotorHelicopter.w_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;

Fus.u = DoubleRotorHelicopter.u;
Fus.v = DoubleRotorHelicopter.v;
Fus.w = DoubleRotorHelicopter.w;
Fus.u_dot = DoubleRotorHelicopter.u_dot;
Fus.v_dot = DoubleRotorHelicopter.v_dot;
Fus.w_dot = DoubleRotorHelicopter.w_dot;
Fus.p = DoubleRotorHelicopter.p;
Fus.q = DoubleRotorHelicopter.q;
Fus.r = DoubleRotorHelicopter.r;
Fus.p_dot = DoubleRotorHelicopter.p_dot;
Fus.q_dot = DoubleRotorHelicopter.q_dot;
Fus.r_dot = DoubleRotorHelicopter.r_dot;

Fus.calculate_force()
Fus.calculate_torque()

%%
test_u_array = -20:1:20;
test_v_array = -20:1:20;
%test_X_matrix = zeros(41,41);
test_Y_matrix = zeros(41,41);
%test_Z_matrix = zeros(41,41);
test_L_matrix = zeros(41,41);
% test_M_matrix = zeros(41,41);
test_N_matrix = zeros(41,41);
[U,V] = meshgrid(test_u_array,test_v_array);

i_v = 1; 
for ele_v  = test_v_array
    i_u = 1;
    for ele_u  = test_u_array
        
        DoubleRotorHelicopter.u         = ele_u;
        DoubleRotorHelicopter.v         = ele_v;
        DoubleRotorHelicopter.w         = 0;
        DoubleRotorHelicopter.u_dot     = 0;
        DoubleRotorHelicopter.v_dot     = 0;
        DoubleRotorHelicopter.w_dot     = 0;
        DoubleRotorHelicopter.p         = 0;
        DoubleRotorHelicopter.q         = 0;
        DoubleRotorHelicopter.r         = 0;
        DoubleRotorHelicopter.p_dot     = 0;
        DoubleRotorHelicopter.q_dot     = 0;
        DoubleRotorHelicopter.r_dot     = 0;

        Fus.u = DoubleRotorHelicopter.u;
        Fus.v = DoubleRotorHelicopter.v;
        Fus.w = DoubleRotorHelicopter.w;
        Fus.u_dot = DoubleRotorHelicopter.u_dot;
        Fus.v_dot = DoubleRotorHelicopter.v_dot;
        Fus.w_dot = DoubleRotorHelicopter.w_dot;
        Fus.p = DoubleRotorHelicopter.p;
        Fus.q = DoubleRotorHelicopter.q;
        Fus.r = DoubleRotorHelicopter.r;
        Fus.p_dot = DoubleRotorHelicopter.p_dot;
        Fus.q_dot = DoubleRotorHelicopter.q_dot;
        Fus.r_dot = DoubleRotorHelicopter.r_dot;

        Fus.calculate_force()
        Fus.calculate_torque()
        
%        test_X_matrix(i_v,i_u) = Fus.X;
        test_Y_matrix(i_v,i_u) = Fus.Y;
%        test_Z_matrix(i_v,i_u) = Fus.Z;
        test_L_matrix(i_v,i_u) = Fus.L;
%         test_M_matrix(i_u,i_v) = Fus.M;
        test_N_matrix(i_v,i_u) = Fus.N;
        
        i_u = i_u + 1;
    end
    i_v = i_v + 1;
end

figure(1)
surf(U,V,abs(test_Y_matrix))
colorbar
view(2)
xlabel('u'); ylabel('v'); zlabel('Y'); legend('Y')

figure(2)
surf(U,V,abs(test_L_matrix))
colorbar
view(2)
xlabel('u'); ylabel('v'); zlabel('L'); legend('L')

figure(3)
surf(U,V,abs(test_N_matrix))
colorbar
view(2)
xlabel('u'); ylabel('v'); zlabel('N'); legend('N')
%% alpha 相关
test_u_array = -20:1:20;
test_w_array = -20:1:20;
test_X_matrix = zeros(41,41);
test_Z_matrix = zeros(41,41);
test_M_matrix = zeros(41,41);
[U,W] = meshgrid(test_u_array,test_w_array);

i_w = 1; 
for ele_w  = test_w_array
    i_u = 1;
    for ele_u  = test_u_array
        
        DoubleRotorHelicopter.u         = ele_u;
        DoubleRotorHelicopter.v         = 0;
        DoubleRotorHelicopter.w         = ele_w;
        DoubleRotorHelicopter.u_dot     = 0;
        DoubleRotorHelicopter.v_dot     = 0;
        DoubleRotorHelicopter.w_dot     = 0;
        DoubleRotorHelicopter.p         = 0;
        DoubleRotorHelicopter.q         = 0;
        DoubleRotorHelicopter.r         = 0;
        DoubleRotorHelicopter.p_dot     = 0;
        DoubleRotorHelicopter.q_dot     = 0;
        DoubleRotorHelicopter.r_dot     = 0;

        Fus.u = DoubleRotorHelicopter.u;
        Fus.v = DoubleRotorHelicopter.v;
        Fus.w = DoubleRotorHelicopter.w;
        Fus.u_dot = DoubleRotorHelicopter.u_dot;
        Fus.v_dot = DoubleRotorHelicopter.v_dot;
        Fus.w_dot = DoubleRotorHelicopter.w_dot;
        Fus.p = DoubleRotorHelicopter.p;
        Fus.q = DoubleRotorHelicopter.q;
        Fus.r = DoubleRotorHelicopter.r;
        Fus.p_dot = DoubleRotorHelicopter.p_dot;
        Fus.q_dot = DoubleRotorHelicopter.q_dot;
        Fus.r_dot = DoubleRotorHelicopter.r_dot;

        Fus.calculate_force()
        Fus.calculate_torque()
        
        test_X_matrix(i_w,i_u) = Fus.X;
%        test_Y_matrix(i_v,i_u) = Fus.Y;
        test_Z_matrix(i_w,i_u) = Fus.Z;
%        test_L_matrix(i_v,i_u) = Fus.L;
        test_M_matrix(i_w,i_u) = Fus.M;
%        test_N_matrix(i_v,i_u) = Fus.N;
        
        i_u = i_u + 1;
    end
    i_w = i_w + 1;
end

figure(1)
surf(U,W,abs(test_X_matrix))
colorbar
view(2)
xlabel('u'); ylabel('w'); zlabel('X'); legend('X')

figure(2)
surf(U,W,abs(test_Z_matrix))
colorbar
view(2)
xlabel('u'); ylabel('w'); zlabel('Z'); legend('Z')

figure(3)
surf(U,W,abs(test_M_matrix))
colorbar
view(2)
xlabel('u'); ylabel('w'); zlabel('M'); legend('M')
