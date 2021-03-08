% compare 2 file and integrate the best part

table1 = readtable('trim_result_redundant_full_jde_03_05_23_50.csv');
table2 = readtable('trim_result_redundant_full_jde_fix_03_08_04_05.csv'); % new
table2(isnan(table2{:,end}),:) = [];

% if size(table1) ~= size(table2)
%     disp("size error")
%     exit;
% end

[number_of_U,~] = size(table2);
for j = 1:number_of_U
    U = table2{j,1};
    
    if table1{find(table1{:,1}==U),end} > table2{j,end}
        table1(find(table1{:,1}==U),:) = table2(j,:);
    end
end
filename = ['trim_result_redundant_full_jde_fix_',datestr(now,'mm_dd_HH_MM'),'.csv'];
writetable(table1,filename);


