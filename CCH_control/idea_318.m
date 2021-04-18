% A integrated analyse
clear all
clc
table_propeller_config = readtable('propeller_config.csv');
run init_build.m

%% Configuration1: no redundant
% objective: generate the dataset, and preconfig theta
% trim_no_redundant.csv
array_U = 0:1:100;
[~,number_of_U] = size(array_U);
matrix_trim_states = zeros(number_of_U,30);
% U,theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2,Prop_theta_0,Prop_isEnable,delta_e,delta_r,theta_1c_diff,theta_1s_diff,v_01,v_02,beta_01,beta_1c1,beta_1s1,beta_02,beta_1c2,beta_1s2,power_total_LowerRotor,power_total_UpperRotor,power_total_Prop,power_total,T_prop,T_rotor
for j = 1:number_of_U
    disp(array_U(j))
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

    zero_force_theta_0_prop = table_propeller_config.zeros_force_theta_0(table_propeller_config.U == ceil(Rotorcraft.DoubleRotorHelicopter.U));

    % x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
    options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',30,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
    cell_InitialStates      = {[0.01,0,0,0,0,0,10,10,1],[0.01,0,0,0,0,0,3,3,1], ...
                                [0.1,0,0,0,0,0,10,10,1],[0.1,0,0,0,0,0,3,3,1], ...
                                [0.2,0,0,0,0,0,10,10,1],[0.2,0,0,0,0,0,3,3,1], ...
                                [0.3,0,0,0,0,0,10,10,1],[0.3,0,0,0,0,0,3,3,1]};
    [x_trim,~,exitflag,~,Rotorcraft,~,power_total] = trim_solve(Rotorcraft, ...
                                    @Aerodynamics_trim_full_9var, ... 
                                    cell_InitialStates, ...
                                    options, ...
                                    2, ...                  % LowerRotor.inteference
                                    2, ...                  % UpperRotor.inteference
                                    zero_force_theta_0_prop, ...         % Prop.theta_0
                                    1, ...                  % Prop.isEnable
                                    1, ...                  % Fus.isEnable
                                    deg2rad(0), ...         % HorStab.delta_e
                                    1, ...                  % HorStab.isEnable
                                    deg2rad(0), ...         % VerStab.delta_r
                                    1, ...                  % VerStab.isEnable
                                    deg2rad(0), ...         % theta_1c_diff
                                    deg2rad(0));            % theta_1s_diff
    if exitflag > 0
        matrix_trim_states(j,:) = [array_U(j) ...
                                    x_trim ...
                                    zero_force_theta_0_prop 1 0 0 0 0 ...
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
                                    Rotorcraft.Prop.T_blade_element ...
                                    Rotorcraft.LowerRotor.T_blade_element+Rotorcraft.UpperRotor.T_blade_element];
    else
        matrix_trim_states(j,:) = [array_U(j) nan*ones(1,29)];
    end
end

VariableNames = {'U','theta_0','theta_diff','theta_1c','theta_1s','theta','phi','v_i1','v_i2','v_iprop', ...
                'Prop_theta_0','Prop_isEnable','delta_e','delta_r','theta_1c_diff','theta_1s_diff', ...
                'v_01','v_02', ...
                'beta_01','beta_1c1','beta_1s1','beta_02','beta_1c2','beta_1s2', ...
                'power_total_LowerRotor', 'power_total_UpperRotor', 'power_total_Prop' ,'power_total',...
                'T_prop','T_rotor'};
table_trim_states = array2table(matrix_trim_states,'VariableNames',VariableNames);
writetable(table_trim_states,'trim_result_no_redundant.csv');

% calculate preconfigured theta
specified_last_theta = deg2rad(-3);
array_theta_preconfig = zeros(size(array_U));
for k = 1:number_of_U
    maxmin = table_trim_states.theta(11) - table_trim_states.theta(end);
    move_up_percent = (specified_last_theta-table_trim_states.theta(end))/maxmin;
    
    array_theta_preconfig(k) = table_trim_states.theta(k) + move_up_percent*(table_trim_states.theta(11)-table_trim_states.theta(k));
end

array_theta_preconfig(1:11) = table_trim_states.theta(1:11);

table_theta_preconfig = array2table([array_U',array_theta_preconfig'],'VariableNames',{'U','theta_preconfig'});
writetable(table_theta_preconfig,'theta_preconfig.csv');

%% Configuration2: only add propeller to satisfy the preconfig theta
% objective: generate trim_redundant_prop.csv
array_U = 0:1:100;
[~,number_of_U] = size(array_U);
matrix_trim_states = zeros(number_of_U,30);
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
                                    Rotorcraft.Prop.T_blade_element ...
                                    Rotorcraft.LowerRotor.T_blade_element+Rotorcraft.UpperRotor.T_blade_element];
end

% write file
VariableNames = {'U','theta_0','theta_diff','theta_1c','theta_1s','theta','phi','v_i1','v_i2','v_iprop', ...
                'Prop_theta_0','Prop_isEnable','delta_e','delta_r','theta_1c_diff','theta_1s_diff', ...
                'v_01','v_02', ...
                'beta_01','beta_1c1','beta_1s1','beta_02','beta_1c2','beta_1s2', ...
                'power_total_LowerRotor', 'power_total_UpperRotor', 'power_total_Prop' ,'power_total',...
                'T_prop','T_rotor'};
table_trim_states = array2table(matrix_trim_states,'VariableNames',VariableNames);
writetable(table_trim_states,'trim_result_redundant_prop.csv');

%% Helper dataset: for different U, vary delta_e, get power and rotor load
U = 81;
array_delta_e           = [deg2rad(-20):deg2rad(0.2):deg2rad(2)];
[~, number_of_delta_e]      = size(array_delta_e); 
array_power_total           = inf(size(array_delta_e));
array_load_rotor            = inf(size(array_delta_e));
matrix_trim_states = zeros(number_of_delta_e,30);

parfor k  = 1:number_of_delta_e
    delta_e = array_delta_e(k); 
    disp(delta_e)

    table_trim_redundant_prop       = readtable('trim_result_redundant_prop.csv');
    table_propeller_config          = readtable('propeller_config.csv');
    table_theta_preconfig           = readtable('theta_preconfig.csv');

    Rotorcraft = struct;
    Rotorcraft.DoubleRotorHelicopter    = DoubleRotorHelicopter;
    Rotorcraft.LowerRotor               = LowerRotor;
    Rotorcraft.UpperRotor               = UpperRotor;
    Rotorcraft.Prop                     = Prop;
    Rotorcraft.Fus                      = Fus;
    Rotorcraft.HorStab                  = HorStab;  
    Rotorcraft.VerStab                  = VerStab;
    Rotorcraft.DoubleRotorHelicopter.U         = U;
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

    nearest_initial         = table_trim_redundant_prop{table_trim_redundant_prop.U == U,2:10};
    zero_force_theta_0_prop = table_propeller_config.zeros_force_theta_0(table_propeller_config.U == U);

    % find the preconfig theta
    theta_specified = table_theta_preconfig.theta_preconfig(table_theta_preconfig.U==U);

    % find the corresp theta_0_prop
    problem = struct;
    problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, [x, delta_e, 0], nearest_initial))^2*1e3;
    problem.x0          = zero_force_theta_0_prop+deg2rad(5);
    problem.lb          = zero_force_theta_0_prop;
    problem.ub          = zero_force_theta_0_prop+deg2rad(25);
    problem.solver      = 'fmincon';
    problem.options     = optimset('Display','off','Algorithm','sqp');
    theta_0_prop = fmincon(problem);

    % get the trim states
    [~,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,[theta_0_prop, delta_e, 0], nearest_initial);

    % record 
    array_power_total(k)    = power_total;  
    array_load_rotor(k)     = Rotorcraft.LowerRotor.T_blade_element+Rotorcraft.UpperRotor.T_blade_element;
    matrix_trim_states(k,:) = [U ...
                                    x_trim ...
                                    theta_0_prop 1 delta_e 0 0 0 ...
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
                                    Rotorcraft.Prop.T_blade_element ...
                                    Rotorcraft.LowerRotor.T_blade_element+Rotorcraft.UpperRotor.T_blade_element];

end

% write file
VariableNames = {'U','theta_0','theta_diff','theta_1c','theta_1s','theta','phi','v_i1','v_i2','v_iprop', ...
                'Prop_theta_0','Prop_isEnable','delta_e','delta_r','theta_1c_diff','theta_1s_diff', ...
                'v_01','v_02', ...
                'beta_01','beta_1c1','beta_1s1','beta_02','beta_1c2','beta_1s2', ...
                'power_total_LowerRotor', 'power_total_UpperRotor', 'power_total_Prop' ,'power_total',...
                'T_prop','T_rotor'};
            
table_trim_states = array2table(matrix_trim_states,'VariableNames',VariableNames);
filename = ['trim_redundant_prop_var_delta_e_U',num2str(U),'.csv'];
writetable(table_trim_states, filename);


%% Configuration3: Add delta_e to make the minimum power
% objective: generate trim_redundant_prop_delta_e_min_power.csv

array_U         = 0:5:100;
[~,number_of_U] = size(array_U);
delta_e_best    = zeros(size(array_U));
power_best      = zeros(size(array_U));
matrix_trim_states = zeros(number_of_U,30);

tic;
disp(datetime)
bar = waitbar(0,'starting');

for m = 1:number_of_U
    U = array_U(m);
    disp(U)    
    if U <= 30  % no delta_e under 30 m/s
        array_delta_e           = 0;
    else
        array_delta_e           = [linspace(deg2rad(-15),deg2rad(-10),12),linspace(deg2rad(-10),deg2rad(1),72)];
    end
    
    [~, number_of_delta_e]      = size(array_delta_e); 
    array_power_total           = inf(size(array_delta_e));

    parfor k  = 1:number_of_delta_e
        delta_e = array_delta_e(k);
        disp(delta_e)
        
        table_trim_redundant_prop       = readtable('trim_result_redundant_prop.csv');
        table_propeller_config          = readtable('propeller_config.csv');
        table_theta_preconfig           = readtable('theta_preconfig.csv');
        
        Rotorcraft = struct;
        Rotorcraft.DoubleRotorHelicopter    = DoubleRotorHelicopter;
        Rotorcraft.LowerRotor               = LowerRotor;
        Rotorcraft.UpperRotor               = UpperRotor;
        Rotorcraft.Prop                     = Prop;
        Rotorcraft.Fus                      = Fus;
        Rotorcraft.HorStab                  = HorStab;  
        Rotorcraft.VerStab                  = VerStab;
        Rotorcraft.DoubleRotorHelicopter.U         = U;
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
       
        nearest_initial         = table_trim_redundant_prop{table_trim_redundant_prop.U == U,2:10};
        zero_force_theta_0_prop = table_propeller_config.zeros_force_theta_0(table_propeller_config.U == U);
        
         % find the preconfig theta
        theta_specified = table_theta_preconfig.theta_preconfig(table_theta_preconfig.U==U);
          
        % find the corresp theta_0_prop
        problem = struct;
        problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, [x, delta_e, 0], nearest_initial))^2*1e3;
        problem.x0          = zero_force_theta_0_prop+deg2rad(5);
        problem.lb          = zero_force_theta_0_prop;
        problem.ub          = zero_force_theta_0_prop+deg2rad(25);
        problem.solver      = 'fmincon';
        problem.options     = optimset('Display','off','Algorithm','sqp');
        theta_0_prop = fmincon(problem);
    
        % get the trim states
        [~,~,~,power_total] = find_theta_given_prop(Rotorcraft,[theta_0_prop, delta_e, 0], nearest_initial);
    
        % record 
        array_power_total(k)    = power_total;  
        
    end
    
    pb = min(array_power_total); % power best
    pb = pb(1);
    power_best(m) = pb;
    
    deb = array_delta_e(array_power_total == power_best(m)); % delta_e best
    deb = deb(1);
    delta_e_best(m) = deb;
    
    % writetable
    Rotorcraft = struct;
    Rotorcraft.DoubleRotorHelicopter    = DoubleRotorHelicopter;
    Rotorcraft.LowerRotor               = LowerRotor;
    Rotorcraft.UpperRotor               = UpperRotor;
    Rotorcraft.Prop                     = Prop;
    Rotorcraft.Fus                      = Fus;
    Rotorcraft.HorStab                  = HorStab;  
    Rotorcraft.VerStab                  = VerStab;
    Rotorcraft.DoubleRotorHelicopter.U         = U;
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
    
    table_trim_redundant_prop       = readtable('trim_result_redundant_prop.csv');
    table_propeller_config          = readtable('propeller_config.csv');
    table_theta_preconfig           = readtable('theta_preconfig.csv');
    
    nearest_initial         = table_trim_redundant_prop{table_trim_redundant_prop.U == U,2:10};
    zero_force_theta_0_prop = table_propeller_config.zeros_force_theta_0(table_propeller_config.U == U);

    % find the preconfig theta
    theta_specified = table_theta_preconfig.theta_preconfig(table_theta_preconfig.U==U);

    % find the corresp theta_0_prop
    problem = struct;
    problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, [x, delta_e_best(m), 0], nearest_initial))^2*1e3;
    problem.x0          = zero_force_theta_0_prop+deg2rad(5);
    problem.lb          = zero_force_theta_0_prop;
    problem.ub          = zero_force_theta_0_prop+deg2rad(25);
    problem.solver      = 'fmincon';
    problem.options     = optimset('Display','iter','Algorithm','sqp');
    theta_0_prop = fmincon(problem);

    % get the trim states
    [theta,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,[theta_0_prop, delta_e_best(m), 0], nearest_initial);

    % record 
    matrix_trim_states(m,:) = [U ...
                                    x_trim ...
                                    theta_0_prop 1 delta_e_best(m) 0 0 0 ...
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
                                    Rotorcraft.Prop.T_blade_element ...
                                    Rotorcraft.LowerRotor.T_blade_element+Rotorcraft.UpperRotor.T_blade_element];
    
    
    % time counting
    number_accumulate = m;
    number_total = number_of_U;
    time_elapsed = toc;
    time_average = time_elapsed/(number_accumulate);
    time_estimated = time_average * (number_total - number_accumulate);
    if number_accumulate < number_total
        str=[num2str(number_accumulate),'/',num2str(number_total),' (TE ',num2str(time_elapsed),'s,ETF ',num2str(time_estimated), 's)'];
    else
        str='Finish!';   
    end
    waitbar(number_accumulate/(number_total),bar,str)                     
                                
end

% write file
VariableNames = {'U','theta_0','theta_diff','theta_1c','theta_1s','theta','phi','v_i1','v_i2','v_iprop', ...
                'Prop_theta_0','Prop_isEnable','delta_e','delta_r','theta_1c_diff','theta_1s_diff', ...
                'v_01','v_02', ...
                'beta_01','beta_1c1','beta_1s1','beta_02','beta_1c2','beta_1s2', ...
                'power_total_LowerRotor', 'power_total_UpperRotor', 'power_total_Prop' ,'power_total',...
                'T_prop','T_rotor'};
table_trim_states = array2table(matrix_trim_states,'VariableNames',VariableNames);
writetable(table_trim_states,'trim_redundant_prop_delta_e_min_power.csv');

%% Configuration4: Add delta_e to make airload increase no more than 5% 
% objective: generate trim_redundant_prop_delta_e_power_5load.csv
load_increase_percent = 0.05;

array_U         = 50;
[~,number_of_U] = size(array_U);
delta_e_best    = zeros(size(array_U));
power_best      = zeros(size(array_U));
matrix_trim_states = zeros(number_of_U,30);

tic;
disp(datetime)
bar = waitbar(0,'starting');

for m = 1:number_of_U
    U = array_U(m);
    disp(U)    
    if U <= 30  % no delta_e under 30 m/s
        array_delta_e           = 0;
    else
        array_delta_e           = linspace(deg2rad(-15),deg2rad(0),80);
    end
    
    [~, number_of_delta_e]      = size(array_delta_e); 
    array_power_total           = inf(size(array_delta_e));
    array_load_rotor            = inf(size(array_delta_e));

    parfor k  = 1:number_of_delta_e
        delta_e = array_delta_e(k);
        disp(delta_e)
        
        table_trim_redundant_prop       = readtable('trim_result_redundant_prop.csv');
        table_propeller_config          = readtable('propeller_config.csv');
        table_theta_preconfig           = readtable('theta_preconfig.csv');
        
        Rotorcraft = struct;
        Rotorcraft.DoubleRotorHelicopter    = DoubleRotorHelicopter;
        Rotorcraft.LowerRotor               = LowerRotor;
        Rotorcraft.UpperRotor               = UpperRotor;
        Rotorcraft.Prop                     = Prop;
        Rotorcraft.Fus                      = Fus;
        Rotorcraft.HorStab                  = HorStab;  
        Rotorcraft.VerStab                  = VerStab;
        Rotorcraft.DoubleRotorHelicopter.U         = U;
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
       
        nearest_initial         = table_trim_redundant_prop{table_trim_redundant_prop.U == U,2:10};
        zero_force_theta_0_prop = table_propeller_config.zeros_force_theta_0(table_propeller_config.U == U);
        
         % find the preconfig theta
        theta_specified = table_theta_preconfig.theta_preconfig(table_theta_preconfig.U==U);
          
        % find the corresp theta_0_prop
        problem = struct;
        problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, [x, delta_e, 0], nearest_initial))^2*1e3;
        problem.x0          = zero_force_theta_0_prop+deg2rad(5);
        problem.lb          = zero_force_theta_0_prop;
        problem.ub          = zero_force_theta_0_prop+deg2rad(25);
        problem.solver      = 'fmincon';
        problem.options     = optimset('Display','off','Algorithm','sqp');
        theta_0_prop = fmincon(problem);
    
        % get the trim states
        [~,~,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,[theta_0_prop, delta_e, 0], nearest_initial);
    
        % record 
        array_power_total(k)    = power_total;  
        array_load_rotor(k)     = Rotorcraft.LowerRotor.T_blade_element+Rotorcraft.UpperRotor.T_blade_element
    end
    
    % read the load of configuration2
    table_trim_redundant_prop = readtable('trim_result_redundant_prop');
    load_rotor_no_delta_e   = table_trim_redundant_prop.T_rotor(table_trim_redundant_prop.U == U);
    load_rotor_max          = load_rotor_no_delta_e*(1+load_increase_percent);
    
    pb = min(array_power_total); % power best
    pb = pb(1);
    if pb <= load_rotor_max % if aorload increase no more than some value(5%)
        power_best(m) = pb;
    else
        % choose the value closest to load_rotor_max
        temp    = abs(array_load_rotor-load_rotor_max);
        index   = temp==min(temp);
        pb      = array_power_total(index);
        pb              = pb(1);
        power_best(m)   = pb;
        disp(power_best(m));
    end
    
    deb = array_delta_e(array_power_total == power_best(m)); % delta_e best
    if length(deb) > 1
        deb = 0;
    else
        delta_e_best(m) = deb;
        disp(delta_e_best(m));
    end
    
    % writetable
    Rotorcraft = struct;
    Rotorcraft.DoubleRotorHelicopter    = DoubleRotorHelicopter;
    Rotorcraft.LowerRotor               = LowerRotor;
    Rotorcraft.UpperRotor               = UpperRotor;
    Rotorcraft.Prop                     = Prop;
    Rotorcraft.Fus                      = Fus;
    Rotorcraft.HorStab                  = HorStab;  
    Rotorcraft.VerStab                  = VerStab;
    Rotorcraft.DoubleRotorHelicopter.U         = U;
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
    
    table_trim_redundant_prop       = readtable('trim_result_redundant_prop.csv');
    table_propeller_config          = readtable('propeller_config.csv');
    table_theta_preconfig           = readtable('theta_preconfig.csv');
    
    nearest_initial         = table_trim_redundant_prop{table_trim_redundant_prop.U == U,2:10};
    zero_force_theta_0_prop = table_propeller_config.zeros_force_theta_0(table_propeller_config.U == U);

    % find the preconfig theta
    theta_specified = table_theta_preconfig.theta_preconfig(table_theta_preconfig.U==U);

    % find the corresp theta_0_prop
    problem = struct;
    problem.objective   = @(x) (theta_specified-find_theta_given_prop(Rotorcraft, [x, delta_e_best(m), 0], nearest_initial))^2*1e3;
    problem.x0          = zero_force_theta_0_prop+deg2rad(5);
    problem.lb          = zero_force_theta_0_prop;
    problem.ub          = zero_force_theta_0_prop+deg2rad(25);
    problem.solver      = 'fmincon';
    problem.options     = optimset('Display','iter','Algorithm','sqp');
    theta_0_prop = fmincon(problem);

    % get the trim states
    [theta,x_trim,Rotorcraft,power_total] = find_theta_given_prop(Rotorcraft,[theta_0_prop, delta_e_best(m), 0], nearest_initial);

    % record 
    matrix_trim_states(m,:) = [U ...
                                    x_trim ...
                                    theta_0_prop 1 delta_e_best(m) 0 0 0 ...
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
                                    Rotorcraft.Prop.T_blade_element ...
                                    Rotorcraft.LowerRotor.T_blade_element+Rotorcraft.UpperRotor.T_blade_element];
    
    
    % time counting
    number_accumulate = m;
    number_total = number_of_U;
    time_elapsed = toc;
    time_average = time_elapsed/(number_accumulate);
    time_estimated = time_average * (number_total - number_accumulate);
    if number_accumulate < number_total
        str=[num2str(number_accumulate),'/',num2str(number_total),' (TE ',num2str(time_elapsed),'s,ETF ',num2str(time_estimated), 's)'];
    else
        str='Finish!';   
    end
    waitbar(number_accumulate/(number_total),bar,str)                     
                                
end

% write file
VariableNames = {'U','theta_0','theta_diff','theta_1c','theta_1s','theta','phi','v_i1','v_i2','v_iprop', ...
                'Prop_theta_0','Prop_isEnable','delta_e','delta_r','theta_1c_diff','theta_1s_diff', ...
                'v_01','v_02', ...
                'beta_01','beta_1c1','beta_1s1','beta_02','beta_1c2','beta_1s2', ...
                'power_total_LowerRotor', 'power_total_UpperRotor', 'power_total_Prop' ,'power_total',...
                'T_prop','T_rotor'};
table_trim_states = array2table(matrix_trim_states,'VariableNames',VariableNames);
%%
writetable(table_trim_states,'trim_redundant_prop_delta_e_power_5load_.csv');

