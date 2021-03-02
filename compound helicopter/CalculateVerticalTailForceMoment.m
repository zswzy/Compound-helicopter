%Vertical tail

% 上下旋翼对垂尾垂向速度影响因子
k_1VS = -1;
k_2VS = -1;
% 动压损失系数，大于0小于等于1，越小损失越大
K_VS = 1; 
% 主旋翼对平尾下洗干扰速度
w_iVS = k_1VS*v_1 + k_2VS*v_2;

u_VS = u + z_VS*q - y_VS*r;
v_VS = v + x_VS*r - z_VS*p;
w_VS = w + y_VS*p - x_VS*q + w_iVS;

% 动压
q_VS = 1/2*K_VS*rho*(u_VS^2+v_VS^2+w_VS^2);

alpha_VS = alpha_0VS + atan(v_VS/u_VS); 
beta_VS = asin(w_VS/sqrt(u_VS^2+v_VS^2+w_VS^2));

% 方向舵效率因子
k_r = sqrt(s_H/s_r)*cos(xi_V);
% 方向舵升力
L_r = -q_VS*s_V*C_XHFitted(alpha_HS,alpha_F)*delta_e*k_r;

%C_YV, C_LV, C_NV data: from NUAA
C_YV = C_YVFitted(alpha_VS, mu);
C_LV = C_LVFitted(alpha_VS, mu);
C_NV = C_NVFitted(alpha_VS, mu);

VerticalTailForce = [cos(alpha_VS)*cos(beta_VS)   -sin(alpha_VS)     -cos(alpha_VS)*sin(beta_VS);
                     sin(alpha_VS)*cos(beta_VS)    cos(alpha_VS)      -sin(alpha_VS)*sin(beta_VS) ;
                     sin(beta_VS)                   0                  cos(beta_VS) ]...
                      *[0;
                        0;
                        0];
X_VS =  VerticalTailForce(1);
Y_VS =  VerticalTailForce(2);
Z_VS =  VerticalTailForce(3);

VerticalTailMoment = [y_VS*Z_VS - z_VS*Y_VS;
                        z_VS*X_VS - x_VS*Z_VS;
                        x_VS*Y_VS - y_VS*X_VS];
L_VS = VerticalTailMoment(1);
M_HS = VerticalTailMoment(2);
N_VS = VerticalTailMoment(3);

