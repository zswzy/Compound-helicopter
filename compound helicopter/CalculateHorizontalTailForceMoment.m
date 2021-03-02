%Horizontal tail

% 上下旋翼对平尾垂向速度影响因子
k_1HS = -1;
k_2HS = -1;
% 动压损失系数，大于0小于等于1，越小损失越大
K_HS = 1; 
% 主旋翼对平尾下洗干扰速度
w_iHS = k_1HS*v_1 + k_2HS*v_2;

u_HS = u + z_HS*q - y_HS*r;
v_HS = v + x_HS*r - z_HS*p;
w_HS = w + y_HS*p - x_HS*q + w_iHS;

% 动压
q_HS = 1/2*K_HS*rho*(u_HS^2+v_HS^2+w_HS^2);

%alpha_0F为机身对平尾下洗
alpha_0F = 0;
alpha_HS = alpha_0F + alpha_0HS + atan(w_HS/u_HS); 
beta_HS = asin(v_HS/sqrt(u_HS^2+v_HS^2+w_HS^2));

% 升降舵效率因子
k_e = sqrt(s_H/s_e)*cos(xi_H);
% 升降舵升力
% L_e = q_HS*s_H*C_XHFitted(alpha_HS,alpha_F)*delta_e*k_e;
L_e = q_HS*s_H*3.5*delta_e*k_e;

% 平尾气动力，力矩
C_LHS=3.5*alpha_HS;        % C_LHS=-3.5*alpha_HS;
C_DHS=0;

F_horizontalstabilizer=[cos(beta_HS)*cos(alpha_HS),-sin(beta_HS)*cos(alpha_HS),-sin(alpha_HS);
                                    sin(beta_HS),                       cos(beta_HS),                       0;
                                    cos(beta_HS)*sin(alpha_HS),-sin(beta_HS)*sin(alpha_HS),cos(alpha_HS)]*...
    [-q_HS*s_H*C_DHS;   0;  -q_HS*s_H*C_LHS];
X_HS=F_horizontalstabilizer(1);
Y_HS=F_horizontalstabilizer(2);
Z_HS=F_horizontalstabilizer(3);

M_horizontalstabilizer=[y_HS*Z_HS-z_HS*Y_HS;z_HS*X_HS-x_HS*Z_HS;x_HS*Y_HS-y_HS*X_HS];
L_HS=M_horizontalstabilizer(1);
M_HS=M_horizontalstabilizer(2);
N_HS=M_horizontalstabilizer(3);


% %C_XH, C_ZH, C_MH data: from NUAA
% C_XH = C_XHFitted(alpha_HS, alpha_F);
% C_ZH = C_ZHFitted(alpha_HS, alpha_F);
% C_MH = C_MHFitted(alpha_HS, alpha_F);
% HorizontalTailForce = [cos(alpha_HS)*cos(beta_HS)   -cos(alpha_HS)*sin(beta_HS) -sin(alpha_HS);
%                       sin(beta_HS)                  cos(beta_HS)                0           ;
%                       sin(alpha_HS)*cos(beta_HS)    -sin(alpha_HS)*sin(beta_HS) cos(alpha_HS)]...
%                       *[-q_HS*s_H*C_XH;
%                         0
%                         -q_HS*s_H*C_ZH + L_e];
% X_HS =  HorizontalTailForce(1);
% Y_HS =  HorizontalTailForce(2);
% Z_HS =  HorizontalTailForce(3);
% 
% HorizontalTailMoment = [y_HS*Z_HS - z_HS*Y_HS;
%                         z_HS*X_HS - x_HS*Z_HS;
%                         x_HS*Y_HS - y_HS*X_HS];
% L_HS = HorizontalTailMoment(1);
% % M_HS = HorizontalTailMoment(2);
% N_HS = HorizontalTailMoment(3);
% 
% HorizontalTailMomentp = [cos(alpha_HS)*cos(beta_HS)   -cos(alpha_HS)*sin(beta_HS) -sin(alpha_HS);
%                       sin(beta_HS)                  cos(beta_HS)                0           ;
%                       sin(alpha_HS)*cos(beta_HS)    -sin(alpha_HS)*sin(beta_HS) cos(alpha_HS)]...
%                       *[0;
%                         q_HS*s_H*R*C_MH
%                         0];
% M_HS = HorizontalTailMomentp(2);
