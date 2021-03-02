function [individual_best,fval_best,output] = jde(problem,options)
%DE 此处显示有关此函数的摘要
%   input: problem
%   problem.dimension: 变量维度
%   problem.objective: 目标函数句柄
%   problem.lb: 下界，如[-1,-1]
%   problem.ub: 上界，如[1,1]
%   options.size_population
%   options.max_generation
%   options.scale_parameter
%   options.scale_parameter_lb
%   options.scale_parameter_ub
%   options.scale_parameter_tau
%   options.crossover_proba
%   options.crossover_proba_tau

%   output: .max_generation, .time_elapsed
tic;
disp('-----------Start differential evolution---------------')
m = problem.dimension;
f = problem.objective;
lb = problem.lb;
ub = problem.ub;

% init
if nargin == 1
    size_population     = 20;
    max_generation      = 10;
    scale_parameter     = 0.7;
    crossover_proba     = 0.3;
else
    size_population     = options.size_population;
    max_generation      = options.max_generation;
    scale_parameter     = options.scale_parameter;
    crossover_proba     = options.crossover_proba;
end
generation          = 1;
matrix_population   = zeros(size_population,m,max_generation);  % 每一代的种群信息
matrix_mutation     = zeros(size_population,m,max_generation);  % 每一代的变异信息
matrix_crossover    = zeros(size_population,m,max_generation);  % 每一代的交叉信息
matrix_fval         = zeros(generation, size_population);       % 每一代每个个体的适应度
array_fval_best     = zeros(1,max_generation)*inf;              % 每一代的最佳适应度
array_fval_mean     = zeros(1,max_generation)*inf;              % 每一代的平均适应度
array_fval_worst    = zeros(1,max_generation)*inf;              % 每一代的最差适应度
array_fval_gap      = zeros(1,max_generation)*inf;              % 每一代最佳与最差适应度之差（绝对值）
matrix_inidivual_best = zeros(generation, m);                   % 每一代的最佳个体



for i = 1:size_population
    matrix_population(i, :,generation) = lb + (ub-lb) .* rand(1,m);
    matrix_fval(generation,i) = f(matrix_population(i, :, generation));
end
array_fval_best(generation) = min(matrix_fval(generation,:));
matrix_inidivual_best(generation,:) = matrix_population(matrix_fval(generation,:)== array_fval_best(generation), :, generation);
fcount = size_population;

fprintf('Genration \t Func-count \t Best f(x) \t Mean f(x) \t Worst f(x) \t Gap f(x) \n')

while generation <= max_generation
    % updata parameter
    if rand() < options.scale_parameter_tau
        scale_parameter = options.scale_parameter_lb + rand()*options.scale_parameter_ub;
    end
    if rand() < options.crossover_proba_tau
        crossover_proba = rand();
    end
    if generation >= 6 ...
            && abs(array_fval_mean(generation-1) - array_fval_mean(generation-2)) < 50 ...
            && abs(array_fval_mean(generation-2) - array_fval_mean(generation-3)) < 50 ...
            && abs(array_fval_mean(generation-3) - array_fval_mean(generation-4)) < 50 ...
            && abs(array_fval_mean(generation-4) - array_fval_mean(generation-5)) < 50 ...
            && abs(array_fval_best(generation-1) - array_fval_best(generation-2)) < 50 ...
            && abs(array_fval_best(generation-2) - array_fval_best(generation-3)) < 50 ...
            && abs(array_fval_best(generation-3) - array_fval_best(generation-4)) < 50 ...
            && abs(array_fval_best(generation-4) - array_fval_best(generation-5)) < 50
        break
    else
        % mutation crossover & selection
        for i = 1:size_population
            % mutation
            % random different individual 
            random_individual_range = randperm(size_population);
            random_individual_range(random_individual_range == i) = [];
            r1 = random_individual_range(1);
            r2 = random_individual_range(2);
            r3 = random_individual_range(3);
            matrix_mutation(i, :, generation) = matrix_population(r1, :, generation) ...
                                                + scale_parameter*(matrix_population(r2, :,generation)-matrix_population(r3, :, generation));
            above_index = matrix_mutation(i, :, generation) > ub;
            matrix_mutation(i, above_index, generation) = ub(above_index);
            below_index = matrix_mutation(i, :, generation) < lb;
            matrix_mutation(i, below_index, generation) = lb(below_index);
            
            % crossover
            matrix_crossover(i, :, generation) = matrix_population(i, :, generation);
            crossover_index = rand(1,m) < crossover_proba;
            matrix_crossover(i, crossover_index, generation) = matrix_mutation(i, crossover_index, generation);
       
            % selection, evaluate
            fval_crossover = f(matrix_crossover(i, :,generation));
            if fval_crossover <= matrix_fval(generation,i)
                matrix_population(i, :,generation+1) = matrix_crossover(i, :,generation);
                matrix_fval(generation + 1, i) = fval_crossover;
            else
                matrix_population(i, :,generation+1) = matrix_population(i, :,generation);
                matrix_fval(generation + 1, i) = matrix_fval(generation,i);
            end
        end

        array_fval_best(generation) = min(matrix_fval(generation,:));
        array_fval_mean(generation) = mean(matrix_fval(generation,:));
        array_fval_worst(generation)= max(matrix_fval(generation,:));
        array_fval_gap(generation)  = abs(array_fval_best(generation)-array_fval_worst(generation));
        
        inidivual_best = matrix_population(matrix_fval(generation,:) == array_fval_best(generation), :, generation);
        inidivual_best = inidivual_best(1,:);
        matrix_inidivual_best(generation,:) = inidivual_best;
        
        fcount = fcount + size_population;
        fprintf('%d \t\t %d \t\t %.3e \t %.3e \t %.3e \t %.3e\n', generation, fcount, array_fval_best(generation),array_fval_mean(generation),array_fval_worst(generation),array_fval_gap(generation))
        generation = generation + 1;
        
    end
    
end

fval_best = min(array_fval_best);
individual_best = matrix_inidivual_best(array_fval_best == fval_best,:);
individual_best = individual_best(1,:);

output = struct;
output.max_generation = generation;
output.time_elapsed = toc;

disp('-----------Finish-------------------------------------')
end


