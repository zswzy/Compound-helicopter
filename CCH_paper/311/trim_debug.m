% 3.12 Rotorcraft trim
clear all
clc
h = 100; % flight altitude
[~,~,~,rho] = atmosisa(h);
%table_trim_states = readtable('trim_result_no_redundant.csv');
%% build object
run init_build.m

%% hover/foward trim X,Y,Z,L,M,N=0,[theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
Rotorcraft.DoubleRotorHelicopter.U         = 0;
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

Rotorcraft.LowerRotor.inteference      = 2;
Rotorcraft.UpperRotor.inteference      = 2;
Rotorcraft.Prop.isEnable               = 0;
Rotorcraft.Prop.theta_0                = 0;
Rotorcraft.Fus.isEnable                = 1;
Rotorcraft.HorStab.delta_e             = 0;
Rotorcraft.HorStab.isEnable            = 1;
Rotorcraft.VerStab.delta_r             = 0;
Rotorcraft.VerStab.isEnable            = 1;
Rotorcraft.LowerRotor.theta_1c_diff    = 0;
Rotorcraft.LowerRotor.theta_1s_diff    = 0;
Rotorcraft.UpperRotor.theta_1c_diff    = 0;
Rotorcraft.UpperRotor.theta_1s_diff    = 0;

theta_0     = rad2deg(15);
theta_diff  = 0;
theta_1c    = 0;
theta_1s    = 0;
theta       = 0;
phi         = 0;
v_i1        = 10; % 下旋翼固有诱导速度
v_i2        = 10; % 上旋翼固有诱导速度
Rotorcraft.DoubleRotorHelicopter.theta = theta;
Rotorcraft.DoubleRotorHelicopter.phi   = phi;
Rotorcraft.DoubleRotorHelicopter.psi   = 0;
Rotorcraft.DoubleRotorHelicopter.calculate_local_variables();
Rotorcraft = copy_states(Rotorcraft);

% 变量赋值/初始化
Rotorcraft.LowerRotor.v_i      = v_i1;
Rotorcraft.LowerRotor.theta_0  = theta_0 - theta_diff;
Rotorcraft.LowerRotor.theta_1c = LowerRotor.direction * (theta_1c - LowerRotor.theta_1c_diff);
Rotorcraft.LowerRotor.theta_1s = theta_1s - LowerRotor.theta_1s_diff;
Rotorcraft.UpperRotor.v_i      = v_i2;
Rotorcraft.UpperRotor.theta_0  = theta_0 + theta_diff;
Rotorcraft.UpperRotor.theta_1c = UpperRotor.direction * (theta_1c + LowerRotor.theta_1c_diff);
Rotorcraft.UpperRotor.theta_1s = theta_1s + LowerRotor.theta_1s_diff;
% 旋翼干扰诱导速度
Rotorcraft.LowerRotor.v_0      = LowerRotor.v_i + LowerRotor.delta_1 * UpperRotor.v_i;
Rotorcraft.UpperRotor.v_0      = UpperRotor.v_i + UpperRotor.delta_2 * LowerRotor.v_i;
%%
% nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U+1),2:9};
nearest_initial_no_redundant = [0.01,0,0,0,0,0,10,10];
% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
cell_InitialStates      = {nearest_initial_no_redundant [0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                            [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                            [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                            [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};

                        
[x_trim,~,~,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                @Aerodynamics_trim_full_8var, ... 
                                cell_InitialStates, ...
                                options, ...
                                2, ...                  % LowerRotor.inteference
                                2, ...                  % UpperRotor.inteference
                                deg2rad(20), ...        % Prop.theta_0
                                0, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                deg2rad(0), ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                deg2rad(0), ...         % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                deg2rad(0), ...         % theta_1c_diff
                                deg2rad(0));            % theta_1s_diff
info_dynamics(Rotorcraft)