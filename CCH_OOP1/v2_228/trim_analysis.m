% 3.6 from 3 .csv file of trimming data, analysis and compare each other

table_trim_no_redundant = readtable('trim_result_no_redundant.csv');
table_trim_redundant_prop = readtable('trim_result_redundent_prop_fmincon_03_04_16_44.csv');
table_trim_redundant_full = readtable('trim_result_redundent_full_jde_03_05_23_50.csv');

array_U = table_trim_no_redundant.U;
power_total_no_redundant = table_trim_no_redundant.power_total/1000;
power_total_redundant_prop = table_trim_redundant_prop.power_total/1000;
power_total_redundant_full = table_trim_redundant_full.power_total/1000;

plot(array_U,power_total_redundant_full,'linewidth',2)
grid on