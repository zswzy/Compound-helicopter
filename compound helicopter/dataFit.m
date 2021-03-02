% 拟合各种数据的文件

%% delta_l, delta_u
Table=readtable("OriginalData.xlsx","Sheet","delta","Range","B1:J3");
amu      = Table{1,:}';
adelta_l = Table{2,:}';
adelta_u = Table{3,:}';
delta_lFitted = fit(amu, adelta_l, 'linearinterp');
delta_uFitted = fit(amu, adelta_u, 'linearinterp');

if show_figure == true
    amu = 0:0.01:0.5;
    [adelta_l,adelta_u] = arrayfun(@(x) CalculateDelta_lu(x,delta_lFitted,delta_uFitted) ,amu);
    plot(amu,adelta_l,'k',amu,adelta_u,'r')
    hold on
    title('\delta_l,\delta_u')
    xlabel('\mu')
    ylabel('\delta_l,\delta_u')
    legend('\delta_l','\delta_u')
    grid on
end
%% X_PR, L_PR
Table=readtable("OriginalData.xlsx","Sheet","propeller","Range","B1:J3");
atheta_0PR = deg2rad(Table{1,:}');
aX_PR = Table{2,:}';
aL_PR = Table{3,:}';
X_PRFitted = fit(atheta_0PR, aX_PR, 'linearinterp');
L_PRFitted = fit(atheta_0PR, aL_PR, 'linearinterp');
if show_figure == true
    figure()
    atheta_0PR = deg2rad(0:0.5:50);
    plot(atheta_0PR,  X_PRFitted(atheta_0PR),atheta_0PR,  L_PRFitted(atheta_0PR))
    legend('X_{PR}','L_{PR}')
    title('X_{PR},L_{PR}')
    xlabel('\theta_{0PR}')
    ylabel('X_{PR},L_{PR}')
    grid on
end

%% C_XH
Table=readtable("OriginalData.xlsx","Sheet","C_XH","Range","A1:H3");
aalpha_HS = deg2rad(Table{1,2:end});
aalpha = deg2rad(Table{2:end,1});
mC_XH = Table{2:end,2:end};
[C_XHFitted, ~] = SurfaceFitting(aalpha_HS, aalpha, mC_XH,fittype);

%% C_ZH
Table=readtable("OriginalData.xlsx","Sheet","C_ZH","Range","A1:F3");
aalpha_HS = deg2rad(Table{1,2:end});
aalpha = deg2rad(Table{2:end,1});
mC_ZH = Table{2:end,2:end};
[C_ZHFitted, ~] = SurfaceFitting(aalpha_HS, aalpha, mC_ZH,fittype);

%% C_MH
Table=readtable("OriginalData.xlsx","Sheet","C_MH","Range","A1:H3");
aalpha_HS = deg2rad(Table{1,2:end});
aalpha = deg2rad(Table{2:end,1});
mC_MH = Table{2:end,2:end};
[C_MHFitted, ~] = SurfaceFitting(aalpha_HS, aalpha, mC_MH,fittype);

%% C_LV
Table=readtable("OriginalData.xlsx","Sheet","C_LV","Range","A1:E3");
aalpha_V = deg2rad(Table{1,2:end});
amu = deg2rad(Table{2:end,1});
mC_LV = Table{2:end,2:end};
[C_LVFitted, ~] = SurfaceFitting(aalpha_V, amu, mC_LV,fittype);

%% C_NV
Table=readtable("OriginalData.xlsx","Sheet","C_NV","Range","A1:E3");
aalpha_V = deg2rad(Table{1,2:end});
amu = deg2rad(Table{2:end,1});
mC_NV = Table{2:end,2:end};
[C_NVFitted, ~] = SurfaceFitting(aalpha_V, amu, mC_NV,fittype);

%% C_YV
Table=readtable("OriginalData.xlsx","Sheet","C_YV","Range","A1:F3");
aalpha_V = deg2rad(Table{1,2:end});
amu = deg2rad(Table{2:end,1});
mC_YV = Table{2:end,2:end};
[C_YVFitted, ~] = SurfaceFitting(aalpha_V, amu, mC_YV,fittype);