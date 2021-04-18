% investigate the rudder efficiency

clear all
clc
table_trim_redundant_prop_delta_e_power_5load = readtable('trim_redundant_prop_delta_e_power_5load.csv');
table_trim_redundant_prop = readtable('trim_result_redundant_prop.csv');
run init_build.m

%%
% calculate from trim table

U = 50;
table_slice = table_trim_redundant_prop_delta_e_power_5load(table_trim_redundant_prop_delta_e_power_5load.U == U,:);
[x_trim,Rotorcraft,~] = calculate_from_trim_table(table_slice);

%info_dynamics(Rotorcraft)

derivatives_primitive = calculate_derivatives_primitive(Rotorcraft,x_trim);
[A,B] = calculate_AB(Rotorcraft,derivatives_primitive);

dNddelta_r = derivatives_primitive.dNddelta_r;
dNdtheta_diff = derivatives_primitive.dNdtheta_diff;
%% iterate 
array_U = 0:5:100;
number_U = length(array_U);
array_dNddelta_r = zeros(size(array_U));
array_dNdtheta_diff = zeros(size(array_U));

array_dXdtheta_diff = zeros(size(array_U));
array_dYdtheta_diff = zeros(size(array_U));
array_dZdtheta_diff = zeros(size(array_U));
array_dLdtheta_diff = zeros(size(array_U));
array_dMdtheta_diff = zeros(size(array_U));

array_dXddelta_r = zeros(size(array_U));
array_dYddelta_r = zeros(size(array_U));
array_dZddelta_r = zeros(size(array_U));
array_dLddelta_r = zeros(size(array_U));
array_dMddelta_r = zeros(size(array_U));

for k = 1:number_U
    U = array_U(k);
    table_slice = table_trim_redundant_prop(table_trim_redundant_prop.U == U,:);
    [x_trim,Rotorcraft,~] = calculate_from_trim_table(table_slice);

    derivatives_primitive = calculate_derivatives_primitive(Rotorcraft,x_trim);

    array_dNddelta_r(k)     = derivatives_primitive.dNddelta_r;
    array_dNdtheta_diff(k)  = derivatives_primitive.dNdtheta_diff;
    
    array_dXdtheta_diff(k) = derivatives_primitive.dXdtheta_diff;
    array_dYdtheta_diff(k) = derivatives_primitive.dYdtheta_diff;
    array_dZdtheta_diff(k) = derivatives_primitive.dZdtheta_diff;
    array_dLdtheta_diff(k) = derivatives_primitive.dLdtheta_diff;
    array_dMdtheta_diff(k) = derivatives_primitive.dMdtheta_diff;
    
    array_dXddelta_r(k) = derivatives_primitive.dXddelta_r;
    array_dYddelta_r(k) = derivatives_primitive.dYddelta_r;
    array_dZddelta_r(k) = derivatives_primitive.dZddelta_r;
    array_dLddelta_r(k) = derivatives_primitive.dLddelta_r;
    array_dMddelta_r(k) = derivatives_primitive.dMddelta_r;
end
%%
figure(1)
plot(array_U,array_dNddelta_r,'linewidth',1.5);
hold on
plot(array_U,array_dNdtheta_diff,'linewidth',1.5);
hold off

grid on;

figure(2)
plot(array_U,array_dXdtheta_diff,'linewidth',1.5);
hold on
plot(array_U,array_dYdtheta_diff,'linewidth',1.5);
hold on
plot(array_U,array_dZdtheta_diff,'linewidth',1.5);
hold on
plot(array_U,array_dLdtheta_diff,'linewidth',1.5);
hold on
plot(array_U,array_dMdtheta_diff,'linewidth',1.5);
hold on
plot(array_U,array_dNdtheta_diff,'linewidth',1.5);
hold off
legend('X','Y','Z','L','M','N')

figure(3)
plot(array_U,array_dXddelta_r,'linewidth',1.5);
hold on
plot(array_U,array_dYddelta_r,'linewidth',1.5);
hold on
plot(array_U,array_dZddelta_r,'linewidth',1.5);
hold on
plot(array_U,array_dLddelta_r,'linewidth',1.5);
hold on
plot(array_U,array_dMddelta_r,'linewidth',1.5);
hold on
plot(array_U,array_dNddelta_r,'linewidth',1.5);
hold off
legend('X','Y','Z','L','M','N')

figure(4)
plot(array_dLdtheta_diff,array_dNdtheta_diff,'linewidth',1.5)
hold on
plot(array_dLddelta_r,array_dNddelta_r,'linewidth',1.5)
hold off

figure(5)
plot(array_U,array_dNdtheta_diff./array_dLdtheta_diff,'linewidth',1.5)
hold on
plot(array_U,array_dNddelta_r./array_dLddelta_r,'linewidth',1.5)
hold off