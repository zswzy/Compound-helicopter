% 3.6 from 3 .csv file of trimming data, analysis and compare each other

table_trim_no_redundant = readtable('trim_result_no_redundant.csv');
table_trim_redundant_prop = readtable('trim_result_redundant_prop.csv');
%table_trim_redundant_full = readtable('trim_result_redundant_full_jde_con_fix_03_09_19_38.csv');

array_U = table_trim_no_redundant.U;
%array_U_full = table_trim_redundant_full.U;
power_total_no_redundant    = table_trim_no_redundant.power_total/1000;
power_total_redundant_prop  = table_trim_redundant_prop.power_total/1000;
%power_total_redundant_full  = table_trim_redundant_full.power_total/1000;

array_theta_no_redundant    = rad2deg(table_trim_no_redundant.theta);
array_theta_redundant_prop  = rad2deg(table_trim_redundant_prop.theta);
%array_theta_redundant_full  = rad2deg(table_trim_redundant_full.theta);

array_phi_no_redundant      = rad2deg(table_trim_no_redundant.phi);
array_phi_redundant_prop    = rad2deg(table_trim_redundant_prop.phi);
%array_phi_redundant_full    = rad2deg(table_trim_redundant_full.phi);

array_theta_0_no_redundant      = rad2deg(table_trim_no_redundant.theta_0);
array_theta_0_redundant_prop    = rad2deg(table_trim_redundant_prop.theta_0);
%array_theta_0_redundant_full    = rad2deg(table_trim_redundant_full.theta_0);

array_theta_diff_no_redundant   = rad2deg(table_trim_no_redundant.theta_diff);
array_theta_diff_redundant_prop = rad2deg(table_trim_redundant_prop.theta_diff);
%array_theta_diff_redundant_full = rad2deg(table_trim_redundant_full.theta_diff);

array_theta_1c_no_redundant     = rad2deg(table_trim_no_redundant.theta_1c);
array_theta_1c_redundant_prop   = rad2deg(table_trim_redundant_prop.theta_1c);
%array_theta_1c_redundant_full   = rad2deg(table_trim_redundant_full.theta_1c);

array_theta_1s_no_redundant     = rad2deg(table_trim_no_redundant.theta_1s);
array_theta_1s_redundant_prop   = rad2deg(table_trim_redundant_prop.theta_1s);
%array_theta_1s_redundant_full   = rad2deg(table_trim_redundant_full.theta_1s);

array_prop_theta_0_redundant_prop = rad2deg(table_trim_redundant_prop.Prop_theta_0);
array_prop_thrust_redundant_prop = table_trim_redundant_prop.T_prop;
%array_prop_theta_0_redundant_full = rad2deg(table_trim_redundant_full.Prop_theta_0);

%array_theta_1c_diff_redundant_full = rad2deg(table_trim_redundant_full.theta_1c_diff);
%array_theta_1s_diff_redundant_full = rad2deg(table_trim_redundant_full.theta_1s_diff);

%array_delta_e_redundant_full = rad2deg(table_trim_redundant_full.delta_e);
%array_delta_r_redundant_full = rad2deg(table_trim_redundant_full.delta_r);

%% if want to calculte others from trim table
array_fuselage_drag_no_redundant = zeros(size(array_U));
[number_of_U,~] = size(array_U);
array_fuselage_drag_redundant_prop = zeros(size(array_U));

for k = 1:number_of_U
    disp(array_U(k))
    [~,Rotorcraft,~] = calculate_from_trim_table(table_trim_no_redundant(k,:));
    array_fuselage_drag_no_redundant(k) = -Rotorcraft.Fus.X;
    
    [~,Rotorcraft,~] = calculate_from_trim_table(table_trim_redundant_prop(k,:));
    array_fuselage_drag_redundant_prop(k) = -Rotorcraft.Fus.X;
end

%%
%[x_trim,Rotorcraft,power_total] = calculate_from_trim_table(table_trim_no_redundant(k,:))
%% draw figure
figure(1)
plot(array_U,power_total_no_redundant,array_U,power_total_redundant_prop,'linewidth',2)
legend('no redundant','redundant propeller')
xlabel('U'); ylabel('Power (kW)'); title('U-Power required');grid on

figure(2)
subplot(1,2,1)
plot(array_U,array_theta_no_redundant,array_U,array_theta_redundant_prop,'linewidth',1.5)
hold on
plot([70 70], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\theta(deg)'); title('U-\theta'); legend('no redundant','redundant propeller');grid on;
subplot(1,2,2)
plot(array_U,array_phi_no_redundant,array_U,array_phi_redundant_prop,'linewidth',1.5)
hold on
plot([70 70], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\phi(deg)'); title('U-\phi'); legend('no redundant','redundant propeller');grid on;

figure(3)
subplot(2,2,1)
plot(array_U,array_theta_0_no_redundant,array_U,array_theta_0_redundant_prop,'linewidth',1.5)
hold on
plot([70 70], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\theta_0 (deg)'); title('U-\theta_0'); legend('no redundant','redundant propeller');grid on;
subplot(2,2,2)
plot(array_U,array_theta_diff_no_redundant,array_U,array_theta_diff_redundant_prop,'linewidth',1.5)
hold on
plot([70 70], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\theta_{diff} (deg)'); title('U-\theta_{diff}'); legend('no redundant','redundant propeller');grid on;
subplot(2,2,3)
plot(array_U,array_theta_1c_no_redundant,array_U,array_theta_1c_redundant_prop,'linewidth',1.5)
hold on
plot([70 70], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\theta_{1c} (deg)'); title('U-\theta_{1c}'); legend('no redundant','redundant propeller');grid on;
subplot(2,2,4)
plot(array_U,array_theta_1s_no_redundant,array_U,array_theta_1s_redundant_prop,'linewidth',1.5)
hold on
plot([70 70], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\theta_{1s} (deg)'); title('U-\theta_{1s}'); legend('no redundant','redundant propeller');grid on;

figure(4)
plot(array_U,array_prop_theta_0_redundant_prop,'linewidth',2)
xlabel('U'); ylabel('\theta_{prop} (deg)'); title('U-\theta_{prop}');grid on

figure(5)
plot(array_U,array_prop_thrust_redundant_prop,'linewidth',2)
xlabel('U'); ylabel('propeller thrust (N)'); title('U-propeller thrust');grid on


figure(7)
plot(array_U,array_fuselage_drag_no_redundant,'linewidth',2)
hold on
plot(array_U,array_fuselage_drag_redundant_prop,'linewidth',2)
hold on
plot(array_U,array_prop_thrust_redundant_prop,'linewidth',2)
hold off
legend('no redundant','redundant prop', 'propeller thrust')
xlabel('U'); ylabel('Fuselage drag/Propeller thrust (N)'); title('U-fuselage drag/propeller thrust'); grid on

%% helper function
% this function help to calculate the rotorcraft from our trim table

function [x_trim,Rotorcraft,power_total] = calculate_from_trim_table(table_slice)
    h = 100; % flight altitude
    [~,~,~,rho] = atmosisa(h);
    
    run init_build.m
    
    Rotorcraft.DoubleRotorHelicopter.U         = table_slice.U;
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

    nearest_initial_no_redundant = table_slice{1,2:10};
    
    options                 = optimset('Display','off','TolFun',1e-15,'Maxiter',30,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
    cell_InitialStates      = {nearest_initial_no_redundant [0.01,0,0,0,0,0,10,10,1],[0.01,0,0,0,0,0,3,3,1], ...
                                [0.1,0,0,0,0,0,10,10,1],[0.1,0,0,0,0,0,3,3,1], ...
                                [0.2,0,0,0,0,0,10,10,1],[0.2,0,0,0,0,0,3,3,1], ...
                                [0.3,0,0,0,0,0,10,10,1],[0.3,0,0,0,0,0,3,3,1]};
    [x_trim,~,~,~,Rotorcraft,~,power_total] = trim_solve(Rotorcraft, ...
                                    @Aerodynamics_trim_full_9var, ... 
                                    cell_InitialStates, ...
                                    options, ...
                                    2, ...                  % LowerRotor.inteference
                                    2, ...                  % UpperRotor.inteference
                                    table_slice.Prop_theta_0, ...        % Prop.theta_0
                                    table_slice.Prop_isEnable, ...                  % Prop.isEnable
                                    1, ...                  % Fus.isEnable
                                    table_slice.delta_e, ...         % HorStab.delta_e
                                    1, ...                  % HorStab.isEnable
                                    table_slice.delta_r, ...         % VerStab.delta_r
                                    1, ...                  % VerStab.isEnable
                                    table_slice.theta_1c_diff, ...         % theta_1c_diff
                                    table_slice.theta_1s_diff);            % theta_1s_diff
end