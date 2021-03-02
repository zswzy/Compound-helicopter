function power_predicted = surrogate_power(surrogate_model,array_predictor)

%VariableNames = {'U','Prop_theta_0','Prop_isEnable','delta_e','delta_r','theta_1c_diff','theta_1s_diff'};

%table_predictor = array2table(array_predictor,'VariableNames',VariableNames);

power_predicted = surrogate_model(array_predictor');

end