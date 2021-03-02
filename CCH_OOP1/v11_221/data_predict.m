clear all; clc;

table_trim_data = readtable('trim_redundant_data_generation_2_22_night.csv');
table_trim_data = table_trim_data(~isnan(table_trim_data{:,end}),:);
%%
trim_predictor = table_trim_data{:,1:7}';
trim_response = table_trim_data{:,end}';

hiddenLayerSize = 20;
net = fitnet(hiddenLayerSize);

net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 15/100;
net.divideParam.testRatio = 15/100;

[net, tr] = train(net, trim_predictor, trim_response);

tInd = tr.testInd;
tstOutputs = net(trim_predictor(:, tInd));
tstPerform = perform(net, trim_response(tInd), tstOutputs)

%% 
Ind_Prop_Enable = find(table_trim_data{:,3});
trim_predictor = table_trim_data{Ind_Prop_Enable,[1,4:7]}';
trim_response = table_trim_data{Ind_Prop_Enable,end}';

hiddenLayerSize = 100;
net = fitnet(hiddenLayerSize);

net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 15/100;
net.divideParam.testRatio = 15/100;

[net, tr] = train(net, trim_predictor, trim_response);

tInd = tr.testInd;
tstOutputs = net(trim_predictor(:, tInd));
tstPerform = perform(net, trim_response(tInd), tstOutputs)

%% DL
table_trim_data = table_trim_data{~isnan(table_trim_data{:,2}),:};