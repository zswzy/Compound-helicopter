% jde parameter analysis

load('jde_investigation3_03-Mar-2021 20:53:16');
array_time_elapsed_average      = mean(matrix_time_elapsed'); % 行向量
array_max_generation_average 	= mean(matrix_max_generation'); % 行向量
array_power_best_average        = mean(matrix_power_best'); % 行向量
[number_population_size,number_sample_each_population] =size(matrix_power_best);
c = linspace(1,10,number_population_size); % color

figure(1)
boxplot(matrix_power_best' /1000,{'4','8','12','16','20','24','28'})
% hold on
% plot(array_power_best_average)
xlabel('size of population'); 
ylabel('best power calculated (kW)');
title('best power calculated (kW) - size of population')
grid on;

figure(2)
%plot(array_time_elapsed_average,'linewidth',2)
boxplot(matrix_time_elapsed',{'4','8','12','16','20','24','28'})
xlabel('size of population'); 
ylabel('average time elapsed (s)')
title('average time elapsed (s) - size of population')
grid on;

figure(3)
%plot(array_max_generation_average,'linewidth',2)
boxplot(matrix_max_generation',{'4','8','12','16','20','24','28'})
xlabel('size of population'); 
ylabel('iteration times (genrations)');
title('iteration times (genrations) - size of population')
grid on;




figure(4)
for k = 1:number_population_size
	for j = 1:number_sample_each_population
		scatter(matrix_time_elapsed(k,j),matrix_power_best(k,j),25,c(k),'filled')
		hold on
	end
end
hold off
xlabel('time elapsed (s)')
ylabel('best power calculated (kW)');
title('best power calculated (kW) - time elapsed (s)')
grid on;

figure(5)
for k = 1:number_population_size
	for j = 1:number_sample_each_population
		scatter(matrix_max_generation(k,j),matrix_power_best(k,j),25,c(k),'filled')
		hold on
	end
end
hold off
xlabel('iteration times (genrations)')
ylabel('best power calculated (kW)');
title('best power calculated (kW) - iteration times (genrations)')
grid on;


