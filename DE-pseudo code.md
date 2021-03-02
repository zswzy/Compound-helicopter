```pseudocode
Minimization problem
Input: 
	problem dimension:  integer m
	objective function: function f
	lower bound: vector lb, dim 1*m
	upper bound: vector ub, dim 1*m
Output:
	fval_best
	individual_best
Initialization:
	number_population = 100*m
	generation = 1
	max_generation = 100
	scale_parameter = 0.5
	crossover_proba = 0.3
	for every individual x(generation,i,:) 
	//i is the index of individual, j is the j-dim value in this individual 
		x(generation,i,j) = lb(j) + rand(0,1)*(ub(j)-lb(j))
Loop:
	while generation <= max_generation
	
	// mutation
	for every individual x(generation,i,:), mutate to v(generation,i,:)
		select 3 other different individual x(generation,r1,:),x(generation,r2,:),x(generation,r3,:)
		v(generation,i,:) = x(generation,r1,:) + scale_parameter*(x(generation,r2,:)-x(generation,r3,:))
	// crossover
	for every mutated individual v(generation,i,:), crossover with x(generation,i,:) to u(generation,i,:)
		for j = 1:m
			if rand(0,1) <= crossover_proba, u(generation,i,j) = v(generation,i,j)
			otherwise, u(generation,i,j) = x(generation,i,j)
	// selection
	for every crossover individual u(i,:)
		if f(u(generation,i,:)) <= f(x(generation,i,:)), x(generation+1,i,:) = u(generation,i,:)
		otherwise, x(generation+1,i,:) = x(generation,i,:)
	
	generation = generation + 1
	fval_best = the minimun value in x(generation,:,:)
end loop


```

