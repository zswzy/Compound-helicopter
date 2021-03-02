%% 数据处理
data = readtable('trim_redundant_data_generation_2_23.csv');
data = data(~isnan(data{:,end}),:);
trim_predictor = data{:,4:7}';
trim_response = data{:,end}';

hiddenLayerSize = 20;
net = fitnet(hiddenLayerSize);

net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 15/100;
net.divideParam.testRatio = 15/100;

[net, tr] = train(net, trim_predictor, trim_response);

tInd = tr.testInd;
tstOutputs = net(trim_predictor(:, tInd));
tstPerform = perform(net, trim_response(tInd), tstOutputs)
%% 使用传统方法得到的最优功率
% 为了避免陷入局部最优，选取多个初始点 [Prop_theta_0, Prop_isEnable, delta_e, delta_r,theta_1c_diff,theta_1s_diff]
cell_redundant_var_initial  = [0,0,0,0];

tic;

problem = struct;
problem.options = optimoptions('fmincon','Display','iter','PlotFcns',[],'algorithm','sqp');
problem.solver = 'fmincon';
problem.objective = @(x) trim_power_prop(Rotorcraft,0,0,x(1),x(2),x(3),x(4));
problem.x0 = cell_redundant_var_initial;
problem.lb = [deg2rad(-25),deg2rad(-30),deg2rad(-1),deg2rad(-1)];
problem.ub = [deg2rad(25),deg2rad(30),deg2rad(1),deg2rad(4.5)];

[redundant_var_best,power_best] = fmincon(problem);

toc

% 获取相应配平变量
% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
cell_InitialStates      = {[0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                            [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                            [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                            [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
[x,~,exitflag,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                @Aerodynamics_trim_full_8var, ... 
                                cell_InitialStates, ...
                                options, ...
                                2, ...                  % LowerRotor.inteference
                                2, ...                  % UpperRotor.inteference
                                0, ...         % Prop.theta_0
                                0, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                redundant_var_best(1), ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                redundant_var_best(2), ...          % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                redundant_var_best(3), ...         % theta_1c_diff
                                redundant_var_best(4));            % theta_1s_diff
%% 用代理模型优化
cell_redundant_var_initial  = [0.4363   -0.3419    0.0175   -0.0175];

tic;

problem = struct;
problem.options = optimoptions('fmincon','Display','iter','PlotFcns',[],'algorithm','sqp');
problem.solver = 'fmincon';
problem.objective = @(x) net([x(1);x(2);x(3);x(4)]);
problem.x0 = cell_redundant_var_initial;
problem.lb = [deg2rad(-25),deg2rad(-30),deg2rad(-1),deg2rad(-1)];
problem.ub = [deg2rad(25),deg2rad(30),deg2rad(1),deg2rad(4.5)];

[redundant_var_best,power_best] = fmincon(problem);

toc

% 获取相应配平变量
% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options                 = optimset('Display','iter','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
cell_InitialStates      = {[0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                            [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                            [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                            [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
[x,~,exitflag,~,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                @Aerodynamics_trim_full_8var, ... 
                                cell_InitialStates, ...
                                options, ...
                                2, ...                  % LowerRotor.inteference
                                2, ...                  % UpperRotor.inteference
                                0, ...         % Prop.theta_0
                                0, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                redundant_var_best(1), ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                redundant_var_best(2), ...          % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                redundant_var_best(3), ...         % theta_1c_diff
                                redundant_var_best(4));            % theta_1s_diff