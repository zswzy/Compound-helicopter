%% data analyse, draw figure

tabel_trim_no_redundant                         = readtable('trim_result_no_redundant.csv');
tabel_trim_redundant_prop                       = readtable('trim_result_redundant_prop.csv');
tabel_trim_redundant_prop_delta_e_min_power     = readtable('trim_redundant_prop_delta_e_min_power.csv');
tabel_trim_redundant_prop_delta_e_power_5load   = readtable('trim_redundant_prop_delta_e_power_5load.csv');
tabel_trim_redundant_prop_var_delta_e_U0        = readtable('trim_redundant_prop_var_delta_e_U0.csv');
tabel_trim_redundant_prop_var_delta_e_U10       = readtable('trim_redundant_prop_var_delta_e_U10.csv');
tabel_trim_redundant_prop_var_delta_e_U30       = readtable('trim_redundant_prop_var_delta_e_U30.csv');
tabel_trim_redundant_prop_var_delta_e_U45       = readtable('trim_redundant_prop_var_delta_e_U45.csv');
tabel_trim_redundant_prop_var_delta_e_U60       = readtable('trim_redundant_prop_var_delta_e_U60.csv');
tabel_trim_redundant_prop_var_delta_e_U80       = readtable('trim_redundant_prop_var_delta_e_U80.csv');
tabel_trim_redundant_prop_var_delta_e_U100      = readtable('trim_redundant_prop_var_delta_e_U100.csv');

%% figure1, x:U, y:power (no redundant, with prop, with delta_e(5%))
figure(1)
plot(tabel_trim_no_redundant.U(1:5:end),tabel_trim_no_redundant.power_total(1:5:end)/1000, 'linewidth',1.5);
hold on
plot(tabel_trim_redundant_prop.U(1:5:end),tabel_trim_redundant_prop.power_total(1:5:end)/1000, 'linewidth',1.5);
hold on
plot(tabel_trim_redundant_prop_delta_e_power_5load.U(1:end),tabel_trim_redundant_prop_delta_e_power_5load.power_total(1:end)/1000, 'linewidth',1.5);
grid on

figure(2)
smooth_plot(tabel_trim_redundant_prop_delta_e_power_5load.U(1:end),tabel_trim_redundant_prop_delta_e_power_5load.power_total(1:end)/1000)
%% helper function: smooth curve
function smooth_plot(x,y)
    xx=linspace(x(1),x(end),100);
    
    order = 10;

    p=polyfit(x,y,order); %其中3为：数据点的个数-1；p为返回多项式的系数；

    yy=polyval(p,xx); %yy为由xx序列根据多项式计算得到的预测值序列；

    plot(x,y,'ro',xx,yy,'b','LineWidth',2); 
end