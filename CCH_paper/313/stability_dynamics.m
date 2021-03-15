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
                                1, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                deg2rad(0), ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                deg2rad(0), ...         % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                deg2rad(0), ...         % theta_1c_diff
                                deg2rad(0));            % theta_1s_diff
info_dynamics(Rotorcraft)

%% Given a flight states (trimmed), get the derivatives of the force and moment, wrt states and control

Rotorcraft.DoubleRotorHelicopter.U         = 20;
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
                                0, ...        % Prop.theta_0
                                0, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                0, ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                0, ...         % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                0, ...         % theta_1c_diff
                                0);            % theta_1s_diff
info_dynamics(Rotorcraft)

derivatives_primitive = calculate_derivatives_primitive(Rotorcraft,x_trim);

[A,B] = calculate_AB(Rotorcraft,derivatives_primitive);
lat = A(5:end,5:end);
eig_lat = eig(lat);
dutch_roll = eig_lat(2);

%% 
%% Given a flight states (trimmed), get the derivatives of the force and moment, wrt states and control

Rotorcraft.DoubleRotorHelicopter.U         = 100;
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
% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
cell_InitialStates      = {nearest_initial_no_redundant [0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                            [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                            [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                            [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
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
for k = 1:80
    Prop_theta_0 = unifrnd(Prop_theta_0_min,Prop_theta_0_max);
    Prop_isEnable = randi(2)-1;
    delta_e = unifrnd(delta_e_min,delta_e_max);
    delta_r = unifrnd(delta_r_min,delta_r_max);
    theta_1c_diff = unifrnd(theta_1c_diff_min,theta_1c_diff_max);
    theta_1s_diff = unifrnd(theta_1s_diff_min,theta_1s_diff_max);
    [x_trim,~,~,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                @Aerodynamics_trim_full_8var, ... 
                                cell_InitialStates, ...
                                options, ...
                                2, ...                  % LowerRotor.inteference
                                2, ...                  % UpperRotor.inteference
                                Prop_theta_0, ...        % Prop.theta_0
                                Prop_isEnable, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                delta_e, ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                delta_r, ...         % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                theta_1c_diff, ...         % theta_1c_diff
                                theta_1s_diff);            % theta_1s_diff
    info_dynamics(Rotorcraft)

    derivatives_primitive = calculate_derivatives_primitive(Rotorcraft,x_trim);

    [A,B] = calculate_AB(Rotorcraft,derivatives_primitive);
    lat = A(5:end,5:end);
    eig_lat = eig(lat);
    dutch_roll = eig_lat(2);
    if imag(dutch_roll) < 0
        dutch_roll = conj(dutch_roll);
    end
    
    figure(1)
    plot(dutch_roll,'bo','linewidth',1)
    grid on
    hold on
end
hold off
%%
Rotorcraft.DoubleRotorHelicopter.U         = 100;
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
% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
cell_InitialStates      = {nearest_initial_no_redundant [0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                            [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                            [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                            [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
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
for k = 1:100
    Prop_theta_0 = unifrnd(Prop_theta_0_min,Prop_theta_0_max);
    Prop_isEnable = randi(2)-1;
    delta_e = unifrnd(delta_e_min,delta_e_max);
    delta_r = unifrnd(delta_r_min,delta_r_max);
    theta_1c_diff = unifrnd(theta_1c_diff_min,theta_1c_diff_max);
    theta_1s_diff = unifrnd(theta_1s_diff_min,theta_1s_diff_max);
    [x_trim,~,~,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                @Aerodynamics_trim_full_8var, ... 
                                cell_InitialStates, ...
                                options, ...
                                2, ...                  % LowerRotor.inteference
                                2, ...                  % UpperRotor.inteference
                                Prop_theta_0, ...        % Prop.theta_0
                                Prop_isEnable, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                delta_e, ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                delta_r, ...         % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                theta_1c_diff, ...         % theta_1c_diff
                                theta_1s_diff);            % theta_1s_diff
    info_dynamics(Rotorcraft)

    derivatives_primitive = calculate_derivatives_primitive(Rotorcraft,x_trim);

    [A,B] = calculate_AB(Rotorcraft,derivatives_primitive);
    lat = A(5:end,5:end);
    eig_lat = eig(lat);
    dutch_roll = eig_lat(2);
    xi = abs(real(dutch_roll)/abs(dutch_roll));
    omega = abs(dutch_roll);
    
    figure(1)
    plot(xi,power_total/1000,'bo','linewidth',1)
    xlabel('\xi'); ylabel('total power (kW)'); title('total power vs \xi');grid on
    hold on
    
    
    figure(2)
    plot(omega,power_total/1000,'bo','linewidth',1)
    xlabel('\omega_n (rad/s)'); ylabel('total power (kW)'); title('total power vs \omega_n');grid on
    hold on
end
hold off
%%
Rotorcraft.DoubleRotorHelicopter.U         = 100;
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
% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
cell_InitialStates      = {nearest_initial_no_redundant [0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                            [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                            [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                            [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
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
for k = 1:80
    Prop_theta_0 = unifrnd(Prop_theta_0_min,Prop_theta_0_max);
    Prop_isEnable = randi(2)-1;
    delta_e = unifrnd(delta_e_min,delta_e_max);
    delta_r = unifrnd(delta_r_min,delta_r_max);
    theta_1c_diff = unifrnd(theta_1c_diff_min,theta_1c_diff_max);
    theta_1s_diff = unifrnd(theta_1s_diff_min,theta_1s_diff_max);
    [x_trim,~,~,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                @Aerodynamics_trim_full_8var, ... 
                                cell_InitialStates, ...
                                options, ...
                                2, ...                  % LowerRotor.inteference
                                2, ...                  % UpperRotor.inteference
                                Prop_theta_0, ...        % Prop.theta_0
                                Prop_isEnable, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                delta_e, ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                delta_r, ...         % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                theta_1c_diff, ...         % theta_1c_diff
                                theta_1s_diff);            % theta_1s_diff
    info_dynamics(Rotorcraft)

    derivatives_primitive = calculate_derivatives_primitive(Rotorcraft,x_trim);

    dMdU = derivatives_primitive.dMdU;
    
    figure(1)
    plot(dMdU,power_total/1000,'bo','linewidth',1)
    xlabel('dM/dU'); ylabel('total power (kW)'); title('total power vs dM/dU');grid on
    hold on
    
  
end
hold off
%%
Rotorcraft.DoubleRotorHelicopter.U         = 100;
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
% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
cell_InitialStates      = {nearest_initial_no_redundant [0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                            [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                            [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                            [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
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
for k = 1:80
    Prop_theta_0 = unifrnd(Prop_theta_0_min,Prop_theta_0_max);
    Prop_isEnable = randi(2)-1;
    delta_e = unifrnd(delta_e_min,delta_e_max);
    delta_r = unifrnd(delta_r_min,delta_r_max);
    theta_1c_diff = unifrnd(theta_1c_diff_min,theta_1c_diff_max);
    theta_1s_diff = unifrnd(theta_1s_diff_min,theta_1s_diff_max);
    [x_trim,~,~,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                @Aerodynamics_trim_full_8var, ... 
                                cell_InitialStates, ...
                                options, ...
                                2, ...                  % LowerRotor.inteference
                                2, ...                  % UpperRotor.inteference
                                Prop_theta_0, ...        % Prop.theta_0
                                Prop_isEnable, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                delta_e, ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                delta_r, ...         % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                theta_1c_diff, ...         % theta_1c_diff
                                theta_1s_diff);            % theta_1s_diff
    info_dynamics(Rotorcraft)

    derivatives_primitive = calculate_derivatives_primitive(Rotorcraft,x_trim);

    dMdv = derivatives_primitive.dMdv;
    
    figure(1)
    plot(dMdv,power_total/1000,'bo','linewidth',1)
    xlabel('dM/dv'); ylabel('total power (kW)'); title('total power vs dM/dv');grid on
    hold on
    
  
end
hold off