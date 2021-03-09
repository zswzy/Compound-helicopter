% 3.7 Rotorcraft trim using surrogate model
clear all
clc
h = 100; % flight altitude
[~,~,~,rho] = atmosisa(h);
table_trim_states = readtable('trim_result_no_redundant.csv');
%% build object
run init_build.m

%% hover/foward trim X,Y,Z,L,M,N=0,[theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
Rotorcraft.DoubleRotorHelicopter.U         = 56.1139764862758;
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

%% redundant trimming file integrate
cell_files = {'trim_redundant_data_generation_03_07_05_17.csv', ...
                'trim_redundant_data_generation_03_07_06_14.csv',...
                'trim_redundant_data_generation_03_07_12_34.csv',...
                'trim_redundant_data_generation_03_07_13_04.csv',...
                'trim_redundant_data_generation_03_07_14_03.csv'};
table_trim_redundant_full = readtable(cell_files{1});

for cell_filename = cell_files(2:end)
    filename = cell_filename{1};
    table_trim_redundant_full = [table_trim_redundant_full; readtable(filename)];
end

% delete nan
table_trim_redundant_full(isnan(table_trim_redundant_full{:,end}),:) = [];
filename = 'trim_redundant_data_integrate.csv';
writetable(table_trim_redundant_full,filename);

train_data = table_trim_redundant_full{:,[1,10:15]};
train_label = table_trim_redundant_full{:,27};
% % delete > 10^6
train_data = train_data(find(train_label < 1e6),:);
train_label = train_label(find(train_label < 1e6));

% normalization

