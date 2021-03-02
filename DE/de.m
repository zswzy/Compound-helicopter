function [individual_best,fval_best] = de(problem)
%DE 此处显示有关此函数的摘要
%   input: problem
%   problem.dimension: 变量维度
%   problem.objective: 目标函数句柄
%   problem.lb: 下界，如[-1,-1]
%   problem.ub: 上界，如[1,1]

m = problem.dimension;
f = problem.objective;
lb = problem.lb;
ub = problem.ub;

% init
size_population     = 20;
generation          = 1;
max_generation      = 50;
scale_parameter     = 0.5;
crossover_proba     = 0.7;
matrix_population   = zeros(max_generation,size_population,m);
matrix_mutation     = zeros(max_generation,size_population,m);
matrix_crossover    = zeros(max_generation,size_population,m);
matrix_fval          = zeros(generation, size_population);
array_fval_best     = zeros(1,max_generation);
matrix_inidivual_best = zeros(generation, m);
for i = 1:size_population
    matrix_population(generation, i, :) = lb + rand(1,m).*(ub-lb);
    matrix_fval(generation,i) = f(matrix_population(generation, i, :));
end
array_fval_best(generation) = min(matrix_fval(generation,:));
matrix_inidivual_best(generation,:) = matrix_population(generation,matrix_fval(generation,:)== array_fval_best(generation), :);

while generation <= max_generation
    % evaluate
    for i = 1:size_population
        matrix_fval(generation, i) = f(matrix_population(generation, i, :));
    end
    array_fval_best(generation) = min(matrix_fval(generation,:));
    matrix_inidivual_best(generation,:) = matrix_population(generation,matrix_fval(generation,:)== array_fval_best(generation), :);
    
    % mutation
    for i = 1:size_population
        % random different individual 
        random_individual_range = randperm(size_population);
        random_individual_range(random_individual_range == i) = [];
        r1 = random_individual_range(1);
        r2 = random_individual_range(2);
        r3 = random_individual_range(3);
        
        matrix_mutation(generation, i, :) = matrix_population(generation, r1, :) ...
                                            + scale_parameter*(matrix_population(generation, r2, :)-matrix_population(generation, r3, :));
    end
    %crossover
    for i = 1:size_population
        matrix_crossover(generation, i, :) = matrix_population(generation, i, :);
        crossover_index = rand(1,m) < crossover_proba;
        matrix_crossover(generation, i, crossover_index) = matrix_mutation(generation, i, crossover_index);
    end
    % selection
    for i = 1:size_population
        if f(matrix_crossover(generation, i, :)) <= matrix_fval(generation,i)
            matrix_population(generation+1, i, :) = matrix_crossover(generation, i, :);
        else
            matrix_population(generation+1, i, :) = matrix_population(generation, i, :);
        end
    end
    generation = generation + 1;
    
end

fval_best = min(array_fval_best);
individual_best = matrix_inidivual_best(array_fval_best == fval_best,:);

end

