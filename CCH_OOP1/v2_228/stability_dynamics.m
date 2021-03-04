% 3.4 dynamics anylisis
clear all
clc
h = 100; % flight altitude
[~,~,~,rho] = atmosisa(h);
table_trim_states = readtable('trim_result_no_redundant.csv');
%% build object
run init_build.m

%% hover/foward trim test X,Y,Z,L,M,N=0,[theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
Rotorcraft.DoubleRotorHelicopter.U         = 80;
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

nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U+1),2:9};
% nearest_initial_no_redundant = [0.01,0,0,0,0,0,10,10];
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

%% Given a flight states (trimmed), get the derivatives of the force and moment, wrt states and control

Rotorcraft.DoubleRotorHelicopter.U         = 10;
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

nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U+1),2:9};
% nearest_initial_no_redundant = [0.01,0,0,0,0,0,10,10];
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
