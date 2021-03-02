% 计算机身的气动力 NUAA 曹燕
% 需要u v w p q r
% 得到FuselageForce,FuselageMoment 向量
% X_F, Y_F, Z_F, L_F, M_F, N_F 标量 体轴

chi1=atan(mu1/(-lambda1));
chi2=atan(mu2/(-lambda2));
k_1F=1.299+0.671*chi1-1.172*chi1^2+0.351*chi1^3;
k_2F=1.299+0.671*chi2-1.172*chi2^2+0.351*chi2^3;
w_iF=k_1F*v1+k_2F*v2;

u_F = u + z_F*q - y_F*r;
v_F = v + x_F*r - z_F*p;
w_F = w + y_F*p - x_F*q + w_iF;

if u_F == 0
    if w_F == 0
        alpha_F = 0;
    else
        alpha_F = pi/2;
    end
else
    alpha_F = atan(w_F/u_F);
end


if (u_F == 0 && v_F == 0)
    beta_F = 0;
else
    beta_F = atan(v_F/sqrt(u_F^2+v_F^2+w_F^2));
end 

q_F = 1/2*rho*(u_F^2+v_F^2+w_F^2);

%% 计算机身气动系数
% 机身气动系数
xs_alpha_CLF=deg2rad([-10.0986,-7.93095,-6.0119,-4.01,-2.00857,-0.00619048,2.0781,4.08,5.99857,8.08381,10.0024,12.0048,14.0905]);
CLF=[-0.0291429,-0.0274286,-0.0231429,-0.0197143,-0.0171429,-0.0128571,-0.0111429,-0.00771429,-0.00428571,-8.57E-04,0.00257143,0.00685714,0.0111429];
C_LF=2*interp1(xs_alpha_CLF,CLF,alpha_F,'spline');

xs_alpha_CMF=deg2rad([-10.3333	-8.25	-6.25	-4.16667	-2.25	-0.25	1.83333	3.83333	5.75	7.83333	9.75	11.75	13.75]);
CMF=[-0.0062069	-0.00551724	-0.00448276	-0.0037931	-0.00275862	-0.00103448	-6.90E-04	6.90E-04	0.00172414	0.00275862	0.00344828	0.00448276	0.00517241];
C_MF=2*interp1(xs_alpha_CMF,CMF,alpha_F,'spline');

xs_CLF=[0.012	0.00942857	0.00514286	0.00171429	-0.00342857	-0.00685714	-0.0102857	-0.0145714	-0.0188571	-0.0231429	-0.0274286];
CDF=[0.0242429	0.0233952	0.0217048	0.0216857	0.0224905	0.0233048	0.0249524	0.0265952	0.0282381	0.029881	0.0323571];
C_DF=2*interp1(xs_CLF,CDF,C_LF/2,'spline');

xs_beta_CYF=deg2rad([-14.9607	-9.95511	-4.85971	-2.48597	0	2.50281	5	10.0954	15.0842]);
CYF=[0.0376543	0.0216049	0.00876543	0.00487654	0	-0.00802469	-0.015	-0.0278395	-0.0407407];
C_YF=2*interp1(xs_beta_CYF,CYF,beta_F,'spline');

xs_beta_CNF=deg2rad([-14.9164	-9.92281	-4.83368	-2.34242	0	2.64564	5.03582	10.2482	15.1519]);
CNF=[0.0382407	0.0244349	0.0127673	0.00690287	0.00E+00	-0.00586439	-0.0128283	-0.0275504	-0.0445327];
C_NF=2*interp1(xs_beta_CNF,CNF,beta_F,'spline');

xs_beta_CRF=deg2rad([-14.2432	-9.25539	-4.26762	-1.72483	0.72016	3.37853	5.71683	10.9135	15.9013]);
CRF=[-8.44E-04	-4.79E-04	-1.15E-04	1.10E-04	2.49E-04	3.94E-04	5.31E-04	9.41E-04	0.00130562];
C_RF=2*interp1(xs_beta_CRF,CRF,beta_F,'spline');


%% 受力计算
FuselageForce=[-q_F*pi*R^2*C_DF;q_F*pi*R^2*C_YF;-q_F*pi*R^2*C_LF];
X_F=FuselageForce(1);
Y_F=FuselageForce(2);
Z_F=FuselageForce(3);


M_fuselage=[q_F*pi*R^2*2*R*C_RF;q_F*pi*R^2*2*R*C_MF;q_F*pi*R^2*2*R*C_NF]+[y_F*Z_F-z_F*Y_F;z_F*X_F-x_F*Z_F;x_F*Y_F-y_F*X_F];
L_F=M_fuselage(1);
M_F=M_fuselage(2);
N_F=M_fuselage(3);

% FuselageForce = [q_F*s_F*Calculate_C_DF(alpha_F,beta_F);
%                             -q_F*s_F*Calculate_C_YF(alpha_F,beta_F);
%                              q_F*s_F*Calculate_C_ZF(alpha_F,beta_F)];
% X_F = FuselageForce(1);
% Y_F = FuselageForce(2);
% Z_F = FuselageForce(3);

% FuselageMoment = [cos(alpha_F)*cos(beta_F)   cos(alpha_F)*sin(beta_F)    -sin(alpha_F)
%                  sin(beta_F)                -cos(beta_F)                0
%                  sin(alpha_F)*cos(beta_F)   sin(alpha_F)*sin(beta_F)    cos(alpha_F)]* ...
%                  [q_F*s_F*l_F*Calculate_C_LF(alpha_F,beta_F);
%                   q_F*s_F*l_F*Calculate_C_MF(alpha_F,beta_F);
%                   q_F*s_F*l_F*Calculate_C_NF(alpha_F,beta_F)] + [-Y_F*z_F + Z_F*y_F;
%                                                                  -Z_F*x_F + X_F*z_F;
%                                                                  Y_F*z_F - X_F*y_F];
% L_F = FuselageMoment(1);
% M_F = FuselageMoment(2);
% N_F = FuselageMoment(3);