% 3.6 from 3 .csv file of trimming data, analysis and compare each other

table_trim_no_redundant = readtable('trim_result_no_redundant.csv');
table_trim_redundant_prop = readtable('trim_result_redundant_prop_simple.csv');
table_trim_redundant_full = readtable('trim_result_redundant_full_jde_fix_03_08_17_49.csv');

array_U = table_trim_no_redundant.U;
power_total_no_redundant    = table_trim_no_redundant.power_total/1000;
power_total_redundant_prop  = table_trim_redundant_prop.power_total/1000;
power_total_redundant_full  = table_trim_redundant_full.power_total/1000;

array_theta_no_redundant    = rad2deg(table_trim_no_redundant.theta);
array_theta_redundant_prop  = rad2deg(table_trim_redundant_prop.theta);
array_theta_redundant_full  = rad2deg(table_trim_redundant_full.theta);

array_phi_no_redundant      = rad2deg(table_trim_no_redundant.phi);
array_phi_redundant_prop    = rad2deg(table_trim_redundant_prop.phi);
array_phi_redundant_full    = rad2deg(table_trim_redundant_full.phi);

array_theta_0_no_redundant      = rad2deg(table_trim_no_redundant.theta_0);
array_theta_0_redundant_prop    = rad2deg(table_trim_redundant_prop.theta_0);
array_theta_0_redundant_full    = rad2deg(table_trim_redundant_full.theta_0);

array_theta_diff_no_redundant   = rad2deg(table_trim_no_redundant.theta_diff);
array_theta_diff_redundant_prop = rad2deg(table_trim_redundant_prop.theta_diff);
array_theta_diff_redundant_full = rad2deg(table_trim_redundant_full.theta_diff);

array_theta_1c_no_redundant     = rad2deg(table_trim_no_redundant.theta_1c);
array_theta_1c_redundant_prop   = rad2deg(table_trim_redundant_prop.theta_1c);
array_theta_1c_redundant_full   = rad2deg(table_trim_redundant_full.theta_1c);

array_theta_1s_no_redundant     = rad2deg(table_trim_no_redundant.theta_1s);
array_theta_1s_redundant_prop   = rad2deg(table_trim_redundant_prop.theta_1s);
array_theta_1s_redundant_full   = rad2deg(table_trim_redundant_full.theta_1s);

array_prop_theta_0_redundant_prop = rad2deg(table_trim_redundant_prop.Prop_theta_0);
array_prop_theta_0_redundant_full = rad2deg(table_trim_redundant_full.Prop_theta_0);


%%
figure(1)
plot(array_U,power_total_no_redundant,array_U,power_total_redundant_prop,'linewidth',2)
legend('no redundant','redundant propeller')
xlabel('U'); ylabel('Power (kW)'); title('U-Power required');grid on

figure(2)
subplot(1,2,1)
plot(array_U,rad2deg(array_theta_no_redundant),array_U,rad2deg(array_theta_redundant_prop),'linewidth',1.5)
hold on
plot([80 80], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\theta(deg)'); title('U-\theta'); legend('no redundant','redundant propeller');grid on;
subplot(1,2,2)
plot(array_U,rad2deg(array_phi_no_redundant),array_U,rad2deg(array_phi_redundant_prop),'linewidth',1.5)
hold on
plot([80 80], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\phi(deg)'); title('U-\phi'); legend('no redundant','redundant propeller');grid on;

figure(3)
subplot(2,2,1)
plot(array_U,rad2deg(array_theta_0_no_redundant),array_U,rad2deg(array_theta_0_redundant_prop),'linewidth',1.5)
hold on
plot([80 80], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\theta_0 (deg)'); title('U-\theta_0'); legend('no redundant','redundant propeller');grid on;
subplot(2,2,2)
plot(array_U,rad2deg(array_theta_diff_no_redundant),array_U,rad2deg(array_theta_diff_redundant_prop),'linewidth',1.5)
hold on
plot([80 80], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\theta_{diff} (deg)'); title('U-\theta_{diff}'); legend('no redundant','redundant propeller');grid on;
subplot(2,2,3)
plot(array_U,rad2deg(array_theta_1c_no_redundant),array_U,rad2deg(array_theta_1c_redundant_prop),'linewidth',1.5)
hold on
plot([80 80], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\theta_{1c} (deg)'); title('U-\theta_{1c}'); legend('no redundant','redundant propeller');grid on;
subplot(2,2,4)
plot(array_U,rad2deg(array_theta_1s_no_redundant),array_U,rad2deg(array_theta_1s_redundant_prop),'linewidth',1.5)
hold on
plot([80 80], get(gca, 'YLim'), '--r', 'LineWidth', 1)
hold off
xlabel('U'); ylabel('\theta_{1s} (deg)'); title('U-\theta_{1s}'); legend('no redundant','redundant propeller');grid on;

figure(4)
plot(array_U,rad2deg(array_prop_theta_0_redundant_prop),'linewidth',2)
legend('redundant propeller')
xlabel('U'); ylabel('\theta_{prop} (deg)'); title('U-\theta_{prop}');grid on