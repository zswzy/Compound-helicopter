% Given a speed, we want to find the propeller pitch to satisfy a specific
% pitch angle
clear all
clc
h = 100; % flight altitude
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

[theta,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft, [zero_force_theta_0_prop, 0, 0], nearest_initial_no_redundant)

info_dynamics(Rotorcraft)

%% let's try set theta = 3deg when U = 0, and find the value of theta_0_prop

% consider the constrain of theta_0_prop, to avoid the thrust reverse
% use fmincon instead of fsolve
Rotorcraft.DoubleRotorHelicopter.U         = 1;
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

problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, [x, 0, 0], nearest_initial_no_redundant))^2*1e3;
problem.x0          = zero_force_theta_0_prop+deg2rad(5);
problem.lb          = zero_force_theta_0_prop;
problem.ub          = zero_force_theta_0_prop+deg2rad(25);
problem.solver      = 'fmincon';
problem.options     = optimset('Display','iter','Algorithm','interior-point');


theta_0_prop = fmincon(problem)


%% let's try to set theta to our preconfig value

Rotorcraft.DoubleRotorHelicopter.U         = 1;
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

problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, [x, 0, 0], nearest_initial_no_redundant))^2*1e3;
problem.x0          = zero_force_theta_0_prop+deg2rad(5);
problem.lb          = zero_force_theta_0_prop;
problem.ub          = zero_force_theta_0_prop+deg2rad(25);
problem.solver      = 'fmincon';
problem.options     = optimset('Display','iter','Algorithm','interior-point');


theta_0_prop = fmincon(problem)

[theta,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,[theta_0_prop, 0, 0], nearest_initial_no_redundant);
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
    problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, [x, 0, 0], nearest_initial_no_redundant))^2*1e3;
    problem.x0          = zero_force_theta_0_prop+deg2rad(5);
    problem.lb          = zero_force_theta_0_prop;
    problem.ub          = zero_force_theta_0_prop+deg2rad(25);
    problem.solver      = 'fmincon';
    problem.options     = optimset('Display','iter','Algorithm','sqp');
    theta_0_prop = fmincon(problem);

    % get the trim states
    [theta,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,[theta_0_prop, 0, 0], nearest_initial_no_redundant);

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
% we vary delta_e also use propeller to satisfy theta, 
% to observe the power variation

% define the variation domain
array_delta_e = linspace(deg2rad(-15),deg2rad(2),10);
[~, number_of_delta_e] = size(array_delta_e);
array_prop_thrust   = zeros(size(array_delta_e));
array_power_total   = zeros(size(array_delta_e));
array_rotor_thrust  = zeros(size(array_delta_e));
array_LOS           = zeros(size(array_delta_e));
array_theta_0           = zeros(size(array_delta_e));

for k  = 1:number_of_delta_e
    table_trim_states = readtable('trim_result_redundant_prop.csv');
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

    % find the preconfig theta
    theta_specified = table_theta_preconfig.theta_preconfig(table_theta_preconfig.U==Rotorcraft.DoubleRotorHelicopter.U);

    % find the corresp theta_0_prop
    problem = struct;
    problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, [x, array_delta_e(k), 0], nearest_initial_no_redundant))^2*1e3;
    problem.x0          = zero_force_theta_0_prop+deg2rad(5);
    problem.lb          = zero_force_theta_0_prop;
    problem.ub          = zero_force_theta_0_prop+deg2rad(25);
    problem.solver      = 'fmincon';
    problem.options     = optimset('Display','iter','Algorithm','sqp');
    theta_0_prop = fmincon(problem);

    % get the trim states
    [theta,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,[theta_0_prop, array_delta_e(k), 0], nearest_initial_no_redundant);

    % record 
    array_prop_thrust(k)   = Rotorcraft.Prop.T_blade_element;
    array_power_total(k)    = power_total;
    array_rotor_thrust(k) = Rotorcraft.LowerRotor.T_blade_element+Rotorcraft.UpperRotor.T_blade_element;
    array_LOS(k) = (abs(Rotorcraft.LowerRotor.L)+abs(Rotorcraft.UpperRotor.L))/(array_rotor_thrust(k)*5.49);
    array_theta_0(k) = Rotorcraft.LowerRotor.theta_0;
end

figure(1)
plot(rad2deg(array_delta_e),array_power_total,'linewidth',1.5);
title('total power vs. \delta_e'); grid on;
figure(2)
plot(rad2deg(array_delta_e),array_prop_thrust,'linewidth',1.5);
title('propeller thrust vs. \delta_e'); grid on;
figure(3)
plot(rad2deg(array_delta_e),array_rotor_thrust,'linewidth',1.5);
title('rotor thrust vs. \delta_e'); grid on;
figure(4)
plot(rad2deg(array_delta_e),rad2deg(array_theta_0),'linewidth',1.5);
title('\theta_0 vs. \delta_e'); grid on;

%% 316 Let's try another strategy
% we vary delta_r also use propeller to satisfy theta, 
% to observe the power variation

% define the variation domain
array_delta_r = linspace(deg2rad(-5),deg2rad(5),5);
[~, number_of_delta_r] = size(array_delta_r);
array_prop_thrust = zeros(size(array_delta_r));
array_power_total = zeros(size(array_delta_r));

for k  = 1:number_of_delta_r
    table_trim_states = readtable('trim_result_redundant_prop.csv');
    table_propeller_config = readtable('propeller_config.csv');
    table_theta_preconfig = readtable('theta_preconfig.csv');
    
    disp(array_delta_r(k))
    
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
    problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, [x, 0, array_delta_r(k)], nearest_initial_no_redundant))^2*1e3;
    problem.x0          = zero_force_theta_0_prop+deg2rad(5);
    problem.lb          = zero_force_theta_0_prop;
    problem.ub          = zero_force_theta_0_prop+deg2rad(25);
    problem.solver      = 'fmincon';
    problem.options     = optimset('Display','iter','Algorithm','sqp');
    theta_0_prop = fmincon(problem);

    % get the trim states
    [theta,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,[theta_0_prop, 0, array_delta_r(k)], nearest_initial_no_redundant);

    % record 
    array_prop_thrust(k)   = Rotorcraft.Prop.T_blade_element;
    array_power_total(k)    = power_total;
end

figure(1)
plot(rad2deg(array_delta_r),array_power_total);
figure(2)
plot(rad2deg(array_delta_r),rad2deg(array_prop_thrust));
%% Find the best delta_e to  decrease power
array_U = 0:5:100;
[~,number_of_U] = size(array_U);
best_delta_e = zeros(size(array_U));
best_power = zeros(size(array_U));
matrix_trim_states = zeros(number_of_U,29);

tic;
disp(datetime)
bar = waitbar(0,'starting');
for m = 1:number_of_U
    disp(array_U(m))
    
    array_delta_e = [linspace(deg2rad(-15),deg2rad(-10),12),linspace(deg2rad(-10),deg2rad(1),72)];
    [~, number_of_delta_e] = size(array_delta_e);
    
    array_power_total = inf(size(array_delta_e));
    
    table_trim_no_redundant = readtable('trim_result_redundant_prop.csv');
    table_propeller_config = readtable('propeller_config.csv');
    table_theta_preconfig = readtable('theta_preconfig.csv');
    
    Rotorcraft = struct;
    Rotorcraft.DoubleRotorHelicopter    = DoubleRotorHelicopter;
    Rotorcraft.LowerRotor               = LowerRotor;
    Rotorcraft.UpperRotor               = UpperRotor;
    Rotorcraft.Prop                     = Prop;
    Rotorcraft.Fus                      = Fus;
    Rotorcraft.HorStab                  = HorStab;  
    Rotorcraft.VerStab                  = VerStab;
    Rotorcraft.DoubleRotorHelicopter.U         = array_U(m);
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
    
    nearest_initial = table_trim_no_redundant{table_trim_no_redundant.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:10};
    zero_force_theta_0_prop = table_propeller_config.zeros_force_theta_0(table_propeller_config.U == ceil(Rotorcraft.DoubleRotorHelicopter.U));
    theta_0_prop_last = zero_force_theta_0_prop+deg2rad(5);
    
    % find the preconfig theta
    theta_specified = table_theta_preconfig.theta_preconfig(table_theta_preconfig.U==Rotorcraft.DoubleRotorHelicopter.U);
    
    for k  = 1:number_of_delta_e
        disp(array_delta_e(k))
        % find the corresp theta_0_prop
        problem = struct;
        problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, [x, array_delta_e(k), 0], nearest_initial))^2*1e3;
        problem.x0          = theta_0_prop_last;
        %problem.x0          = zero_force_theta_0_prop+deg2rad(5);
        problem.lb          = zero_force_theta_0_prop;
        problem.ub          = zero_force_theta_0_prop+deg2rad(25);
        problem.solver      = 'fmincon';
        problem.options     = optimset('Display','off','Algorithm','sqp');
        theta_0_prop = fmincon(problem);
        
        theta_0_prop_last = theta_0_prop;
    
        % get the trim states
        [~,~,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,[theta_0_prop, array_delta_e(k), 0], nearest_initial_no_redundant);
    
        % record 
        array_power_total(k)    = power_total;
        
        number_accumulate = (m-1)*number_of_delta_e+k;
        time_elapsed = toc;
        time_average = time_elapsed/(number_accumulate);
        time_estimated = time_average * (number_of_U*number_of_delta_e - number_accumulate);
        if number_accumulate < number_of_U*number_of_delta_e
            str=[num2str(number_accumulate),'/',num2str(number_of_U*number_of_delta_e),' (TE ',num2str(time_elapsed),'s,ETF ',num2str(time_estimated), 's)'];
        else
            str='Finish';   
        end
        waitbar(number_accumulate/(number_of_U*number_of_delta_e),bar,str)                     
        
%         if k > 2 && array_power_total(k) - array_power_total(k-1) > 0
%             break;
%         end
    end
    
    bp = min(array_power_total);
    bp = bp(1);
    best_power(m) = bp;
    bd = array_delta_e(array_power_total == best_power(m));
    bd = bd(1);
    best_delta_e(m) = bd;
                                
end

% write file
VariableNames = {'U','delta_e','power'};
table_trim_states = array2table([array_U',best_delta_e',best_power'],'VariableNames',VariableNames);
writetable(table_trim_states,'trim_result_redundant_prop_delta_e.csv');

%% Find the best delta_e to  decrease power (same as previous section but using parfor)
array_U = 0:5:100;
[~,number_of_U] = size(array_U);
best_delta_e = zeros(size(array_U));
best_power = zeros(size(array_U));
matrix_trim_states = zeros(number_of_U,29);

tic;
disp(datetime)
bar = waitbar(0,'starting');
for m = 1:number_of_U
    disp(array_U(m))
    
    array_delta_e = [linspace(deg2rad(-15),deg2rad(-10),12),linspace(deg2rad(-10),deg2rad(1),72)];
    [~, number_of_delta_e] = size(array_delta_e);
    
    array_power_total = inf(size(array_delta_e));

    parfor k  = 1:number_of_delta_e
        table_trim_no_redundant = readtable('trim_result_redundant_prop.csv');
        table_propeller_config = readtable('propeller_config.csv');
        table_theta_preconfig = readtable('theta_preconfig.csv');
        
        Rotorcraft = struct;
        Rotorcraft.DoubleRotorHelicopter    = DoubleRotorHelicopter;
        Rotorcraft.LowerRotor               = LowerRotor;
        Rotorcraft.UpperRotor               = UpperRotor;
        Rotorcraft.Prop                     = Prop;
        Rotorcraft.Fus                      = Fus;
        Rotorcraft.HorStab                  = HorStab;  
        Rotorcraft.VerStab                  = VerStab;
        Rotorcraft.DoubleRotorHelicopter.U         = array_U(m);
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
       
        nearest_initial = table_trim_no_redundant{table_trim_no_redundant.U == fix(Rotorcraft.DoubleRotorHelicopter.U),2:10};
        zero_force_theta_0_prop = table_propeller_config.zeros_force_theta_0(table_propeller_config.U == ceil(Rotorcraft.DoubleRotorHelicopter.U));
        
         % find the preconfig theta
        theta_specified = table_theta_preconfig.theta_preconfig(table_theta_preconfig.U==Rotorcraft.DoubleRotorHelicopter.U);
    
        disp(array_delta_e(k))
        % find the corresp theta_0_prop
        problem = struct;
        problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, [x, array_delta_e(k), 0], nearest_initial))^2*1e3;
        problem.x0          = zero_force_theta_0_prop+deg2rad(5);
        problem.lb          = zero_force_theta_0_prop;
        problem.ub          = zero_force_theta_0_prop+deg2rad(25);
        problem.solver      = 'fmincon';
        problem.options     = optimset('Display','off','Algorithm','sqp');
        theta_0_prop = fmincon(problem);
        
        theta_0_prop_last = theta_0_prop;
    
        % get the trim states
        [~,~,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,[theta_0_prop, array_delta_e(k), 0], nearest_initial_no_redundant);
    
        % record 
        array_power_total(k)    = power_total;
        
        
        
    end
    
    bp = min(array_power_total);
    bp = bp(1);
    best_power(m) = bp;
    bd = array_delta_e(array_power_total == best_power(m));
    bd = bd(1);
    best_delta_e(m) = bd;
    
    number_accumulate = m;
    number_total = number_of_U;
    time_elapsed = toc;
    time_average = time_elapsed/(number_accumulate);
    time_estimated = time_average * (number_total - number_accumulate);
    if number_accumulate < number_total
        str=[num2str(number_accumulate),'/',num2str(number_total),' (TE ',num2str(time_elapsed),'s,ETF ',num2str(time_estimated), 's)'];
    else
        str='Finish';   
    end
    waitbar(number_accumulate/(number_total),bar,str)                     
                                
end

% write file
VariableNames = {'U','delta_e','power'};
table_trim_states = array2table([array_U',best_delta_e',best_power'],'VariableNames',VariableNames);
writetable(table_trim_states,'trim_result_redundant_prop_delta_e.csv');

%% generate all data from previous section
table_part_trim_delta_e = readtable('trim_result_redundant_prop_delta_e.csv');


array_U = table_part_trim_delta_e.U';
array_delta_e = table_part_trim_delta_e.delta_e';
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
    problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, [x, array_delta_e(k), 0], nearest_initial_no_redundant))^2*1e3;
    problem.x0          = zero_force_theta_0_prop+deg2rad(5);
    problem.lb          = zero_force_theta_0_prop;
    problem.ub          = zero_force_theta_0_prop+deg2rad(25);
    problem.solver      = 'fmincon';
    problem.options     = optimset('Display','iter','Algorithm','sqp');
    theta_0_prop = fmincon(problem);

    % get the trim states
    [theta,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,[theta_0_prop, array_delta_e(k), 0], nearest_initial_no_redundant);

    % record 
    matrix_trim_states(k,:) = [array_U(k) ...
                                    x_trim ...
                                    theta_0_prop 1 array_delta_e(k) 0 0 0 ...
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
writetable(table_trim_states,'trim_result_redundant_prop_delta_e_full.csv');

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

%% 316 we are going to reformulate the problem: vary theta_0_prop and delta_e, to see the variation of theta and power
array_delta_e = linspace(deg2rad(-5),deg2rad(5),5);
[~, number_of_delta_e] = size(array_delta_e);
array_theta_0_prop = linspace(zero_force_theta_0_prop,zero_force_theta_0_prop+deg2rad(25),6);
[~, number_of_theta_0_prop] = size(array_theta_0_prop);
array_prop_thrust = zeros(size(array_delta_e));
array_power_total = zeros(size(array_delta_e));
matrix_power = zeros(number_of_delta_e,number_of_theta_0_prop);
matrix_theta = zeros(number_of_delta_e,number_of_theta_0_prop);

for k = 1:number_of_delta_e
    disp(array_delta_e(k))
    
    for m = 1:number_of_theta_0_prop
        
        table_trim_states = readtable('trim_result_redundant_prop.csv');
        table_propeller_config = readtable('propeller_config.csv');
        table_theta_preconfig = readtable('theta_preconfig.csv');
    
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

        [theta,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft, [array_theta_0_prop(m), array_delta_e(k), 0], nearest_initial_no_redundant);

        matrix_power(k,m) = power_total;
        matrix_theta(k,m) = theta;
        
        figure(1)
        scatter(rad2deg(theta),power_total/1000,'bo'); grid on; xlabel('\theta (deg)');ylabel('power(kW)'); title('\theta vs power');
        hold on
    end
end
hold off


%% helper function
function [theta,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,redundant,nearest_initial_no_redundant)
    % x_trim = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2,v_iprop]
    % redundant  = [theta_0_prop,delta_e]
    theta_0_prop    = redundant(1);
    delta_e         = redundant(2);
    delta_r         = redundant(3);
    
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
                                    delta_r, ...         % VerStab.delta_r
                                    1, ...                  % VerStab.isEnable
                                    deg2rad(0), ...         % theta_1c_diff
                                    deg2rad(0));            % theta_1s_diff
     theta = x_trim(5);
end



