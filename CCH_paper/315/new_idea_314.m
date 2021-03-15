% Given a speed, we want to find the propeller pitch to satisfy a specific
% pitch angle
clear all
clc
h = 2000; % flight altitude
[~,~,~,rho] = atmosisa(h);
table_trim_states = readtable('trim_result_no_redundant.csv');
table_propeller_config = readtable('propeller_config.csv');
table_theta_preconfig = readtable('theta_preconfig.csv');
%% build object
run init_build.m

%% test program with zeros force theta_0_prop
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

nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:10};
zero_force_theta_0_prop = table_propeller_config.zeros_force_theta_0(table_propeller_config.U == ceil(Rotorcraft.DoubleRotorHelicopter.U));

[theta,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft, zero_force_theta_0_prop, 0, nearest_initial_no_redundant)

info_dynamics(Rotorcraft)

%% let's try set theta = 3deg when U = 0, and find the value of theta_0_prop

% consider the constrain of theta_0_prop, to avoid the thrust reverse
% use fmincon instead of fsolve
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

nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:10};
zero_force_theta_0_prop = table_propeller_config.zeros_force_theta_0(table_propeller_config.U == ceil(Rotorcraft.DoubleRotorHelicopter.U));

theta_specified = 0.0562;

problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, x, 0, nearest_initial_no_redundant))^2*1e3;
problem.x0          = zero_force_theta_0_prop+deg2rad(5);
problem.lb          = zero_force_theta_0_prop;
problem.ub          = zero_force_theta_0_prop+deg2rad(25);
problem.solver      = 'fmincon';
problem.options     = optimset('Display','iter','Algorithm','interior-point');


theta_0_prop = fmincon(problem)


%% let's try to set theta to our preconfig value

Rotorcraft.DoubleRotorHelicopter.U         = 70;
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

nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:10};
zero_force_theta_0_prop = table_propeller_config.zeros_force_theta_0(table_propeller_config.U == ceil(Rotorcraft.DoubleRotorHelicopter.U));

% find the preconfig theta
theta_specified = table_theta_preconfig.theta_preconfig(table_theta_preconfig.U==Rotorcraft.DoubleRotorHelicopter.U);

problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, x, 0, nearest_initial_no_redundant))^2*1e3;
problem.x0          = zero_force_theta_0_prop+deg2rad(5);
problem.lb          = zero_force_theta_0_prop;
problem.ub          = zero_force_theta_0_prop+deg2rad(25);
problem.solver      = 'fmincon';
problem.options     = optimset('Display','iter','Algorithm','interior-point');


theta_0_prop = fmincon(problem)

[theta,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,theta_0_prop, 0, nearest_initial_no_redundant);
info_dynamics(Rotorcraft)

%% 3.15 let's iterate the U and find every theta_0_prop, and value of X_prop

array_U = 0:1:100;
[~,number_of_U] = size(array_U);
matrix_trim_states = zeros(number_of_U,29);
% U,theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2,Prop_theta_0,Prop_isEnable,delta_e,delta_r,theta_1c_diff,theta_1s_diff,v_01,v_02,beta_01,beta_1c1,beta_1s1,beta_02,beta_1c2,beta_1s2,power_total_LowerRotor,power_total_UpperRotor,power_total_Prop,power_total,T_prop

parfor k = 1:number_of_U
    table_trim_states = readtable('trim_result_no_redundant.csv');
    table_propeller_config = readtable('propeller_config.csv');
    table_theta_preconfig = readtable('theta_preconfig.csv');
    
    disp(array_U(k))
    
    Rotorcraft = struct;
    Rotorcraft.DoubleRotorHelicopter    = DoubleRotorHelicopter;
    Rotorcraft.LowerRotor               = LowerRotor;
    Rotorcraft.UpperRotor               = UpperRotor;
    Rotorcraft.Prop                     = Prop;
    Rotorcraft.Fus                      = Fus;
    Rotorcraft.HorStab                  = HorStab;  
    Rotorcraft.VerStab                  = VerStab;
    Rotorcraft.DoubleRotorHelicopter.U         = array_U(k);
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
    
    nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:10};
    zero_force_theta_0_prop = table_propeller_config.zeros_force_theta_0(table_propeller_config.U == ceil(Rotorcraft.DoubleRotorHelicopter.U));

    % find the preconfig theta
    theta_specified = table_theta_preconfig.theta_preconfig(table_theta_preconfig.U==Rotorcraft.DoubleRotorHelicopter.U);

    % find the corresp theta_0_prop
    problem = struct;
    problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, x, 0, nearest_initial_no_redundant))^2*1e3;
    problem.x0          = zero_force_theta_0_prop+deg2rad(5);
    problem.lb          = zero_force_theta_0_prop;
    problem.ub          = zero_force_theta_0_prop+deg2rad(25);
    problem.solver      = 'fmincon';
    problem.options     = optimset('Display','iter','Algorithm','sqp');
    theta_0_prop = fmincon(problem);

    % get the trim states
    [theta,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,theta_0_prop, 0, nearest_initial_no_redundant);

    % record 
    matrix_trim_states(k,:) = [array_U(k) ...
                                    x_trim ...
                                    theta_0_prop 1 0 0 0 0 ...
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
                                    power_total ...
                                    Rotorcraft.Prop.T_blade_element];
end

% write file
VariableNames = {'U','theta_0','theta_diff','theta_1c','theta_1s','theta','phi','v_i1','v_i2','v_iprop', ...
                'Prop_theta_0','Prop_isEnable','delta_e','delta_r','theta_1c_diff','theta_1s_diff', ...
                'v_01','v_02', ...
                'beta_01','beta_1c1','beta_1s1','beta_02','beta_1c2','beta_1s2', ...
                'power_total_LowerRotor', 'power_total_UpperRotor', 'power_total_Prop' ,'power_total',...
                'T_prop'};
table_trim_states = array2table(matrix_trim_states,'VariableNames',VariableNames);
writetable(table_trim_states,'trim_result_redundant_prop.csv');

% if want to see some results...
figure(1)
plot(table_trim_states.U,table_trim_states.power_total);
xlabel('U'); ylabel('Power'); title('U-Power required'); grid on;
figure(2)
plot(table_trim_states.U,table_trim_states.T_prop);
xlabel('U'); ylabel('Propeller thrust (N)'); title('U-Propeller thrust'); grid on;
figure(3)
plot(table_trim_states.U,rad2deg(table_trim_states.theta));
xlabel('U'); ylabel('pitch angle (deg)'); title('U-pitch angle'); grid on;
%% Let's try another strategy
% we vary at the same time delta_e and theta_0_prop, to satisfy theta, but
% also to observe the power variation

% define the variation domain
array_delta_e = linspace(deg2rad(-5),deg2rad(20),8);
[~, number_of_delta_e] = size(array_delta_e);
array_theta_0_prop = zeros(size(array_delta_e));
array_power_total = zeros(size(array_delta_e));

for k  = 1:number_of_delta_e
    table_trim_states = readtable('trim_result_no_redundant.csv');
    table_propeller_config = readtable('propeller_config.csv');
    table_theta_preconfig = readtable('theta_preconfig.csv');
    
    disp(array_delta_e(k))
    
    Rotorcraft = struct;
    Rotorcraft.DoubleRotorHelicopter    = DoubleRotorHelicopter;
    Rotorcraft.LowerRotor               = LowerRotor;
    Rotorcraft.UpperRotor               = UpperRotor;
    Rotorcraft.Prop                     = Prop;
    Rotorcraft.Fus                      = Fus;
    Rotorcraft.HorStab                  = HorStab;  
    Rotorcraft.VerStab                  = VerStab;
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
    
    nearest_initial_no_redundant = table_trim_states{table_trim_states.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:10};
    zero_force_theta_0_prop = table_propeller_config.zeros_force_theta_0(table_propeller_config.U == ceil(Rotorcraft.DoubleRotorHelicopter.U));

    % find the preconfig theta
    theta_specified = table_theta_preconfig.theta_preconfig(table_theta_preconfig.U==Rotorcraft.DoubleRotorHelicopter.U);

    % find the corresp theta_0_prop
    problem = struct;
    problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, x, 0, nearest_initial_no_redundant))^2*1e3;
    problem.x0          = zero_force_theta_0_prop+deg2rad(5);
    problem.lb          = zero_force_theta_0_prop;
    problem.ub          = zero_force_theta_0_prop+deg2rad(25);
    problem.solver      = 'fmincon';
    problem.options     = optimset('Display','iter','Algorithm','sqp');
    theta_0_prop = fmincon(problem);

    % get the trim states
    [theta,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,theta_0_prop, array_delta_e(k), nearest_initial_no_redundant);

    % record 
    array_theta_0_prop(k)   = theta_0_prop;
    array_power_total(k)    = power_total;
end
    
plot(rad2deg(array_delta_e),array_power_total);
plot(rad2deg(array_delta_e),rad2deg(array_theta_0_prop));

%% helper function
function [theta,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,theta_0_prop,delta_e,nearest_initial_no_redundant)
    % x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2,v_iprop]
    options                 = optimset('Display','off','TolFun',1e-15,'Maxiter',30,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
    cell_InitialStates      = {nearest_initial_no_redundant [0.01,0,0,0,0,0,10,10,1],[0.01,0,0,0,0,0,3,3,1], ...
                                [0.1,0,0,0,0,0,10,10,1],[0.1,0,0,0,0,0,3,3,1], ...
                                [0.2,0,0,0,0,0,10,10,1],[0.2,0,0,0,0,0,3,3,1], ...
                                [0.3,0,0,0,0,0,10,10,1],[0.3,0,0,0,0,0,3,3,1]};
    [x_trim,~,~,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                    @Aerodynamics_trim_full_9var, ... 
                                    cell_InitialStates, ...
                                    options, ...
                                    2, ...                  % LowerRotor.inteference
                                    2, ...                  % UpperRotor.inteference
                                    theta_0_prop, ...        % Prop.theta_0
                                    1, ...                  % Prop.isEnable
                                    1, ...                  % Fus.isEnable
                                    delta_e, ...         % HorStab.delta_e
                                    1, ...                  % HorStab.isEnable
                                    deg2rad(0), ...         % VerStab.delta_r
                                    1, ...                  % VerStab.isEnable
                                    deg2rad(0), ...         % theta_1c_diff
                                    deg2rad(0));            % theta_1s_diff
     theta = x_trim(5);
end



