function [] = PropellerCarac()

    atheta_PR = 20:40;
    
    aX_PR = [];
    aL_PR = [];
    for theta_PR  = atheta_PR
        [F,M] = PropellerForceMoment(theta_PR);
        aX_PR = [aX_PR F];
        aL_PR = [aL_PR M];
    end
    subplot(2,1,1);
    plot(atheta_PR, aX_PR)
    xlabel('theta_PR(deg)');ylabel('N');grid on;

    subplot(2,1,2);
    plot(atheta_PR, aL_PR)
    xlabel('theta_PR(deg)');ylabel('Lm');grid on;
end

function [F,M] = PropellerForceMoment(thetaPR)
global theta_PR X_PR L_PR

    theta_PR = deg2rad(thetaPR);
    [x,~,~,~] = fsolve(@PropellerEqua,[1]);
    v0_PR = x;

    F = X_PR;
    M = L_PR;

end



function F = PropellerEqua(v0_PR)
% 悬停状态下推进桨推力特性

%% 推进螺旋桨 X_PR Y_PR Z_PR L_PR M_PR N_PR
% 配平量  尾桨的根部安装角theta_PR v_0PR
global K_PR x_PR y_PR z_PR Omega_PR R_PR rho a_PR c_PR I_beta_PR kappa sigma_PR theta_tPR theta_PR X_PR L_PR
% chi1=atan(mu_1/(-lambda_1));
% chi2=atan(mu_2/(-lambda_2));
% k_1PR=1.299+0.671*chi1-1.172*chi1^2+0.351*chi1^3;
% k_2PR=1.299+0.671*chi2-1.172*chi2^2+0.351*chi2^3;
% w_iPR=k_1PR*v_1+k_2PR*v_2;
v_1 = 5; v_2 = 10;
w_iPR=-0.1*v_1-0.1*v_2;

u = 0; v = 0; w = 0; p = 0; q = 0; r = 0;
% 推进桨气动中心在机体坐标系下速度
vol_PR=[u*sqrt(K_PR)+z_PR*q-y_PR*r;v*sqrt(K_PR)+x_PR*r-z_PR*p;w*sqrt(K_PR)+y_PR*p-x_PR*q+w_iPR];
u_PR=vol_PR(1);
v_PR=vol_PR(2);
w_PR=vol_PR(3);

% disp(vol_PR)

B_u_PR=u_PR/(Omega_PR*R_PR);
B_v_PR=v_PR/(Omega_PR*R_PR);
B_w_PR=w_PR/(Omega_PR*R_PR);

mu_PR=sqrt(B_v_PR^2+B_w_PR^2);
lambda_0PR=-B_u_PR;        %  lambda_0PR=B_u_PR;


%  推进桨挥舞锥度角，拉力系数，反扭矩系数
gamma_PR=rho*a_PR*c_PR*R_PR^4/I_beta_PR;
a0_PR=kappa*gamma_PR/2*(1/4*(1+mu_PR^2)*theta_PR+(1/5+1/6*mu_PR^2)*theta_tPR-1/3*(v0_PR-lambda_0PR));
C_TPR=a_PR*sigma_PR*((1/3+1/2*mu_PR^2)*theta_PR+1/4*(1+mu_PR^2)*theta_tPR-1/2*(v0_PR-lambda_0PR));

C_DPR=0.009+0.3*(6*C_TPR/a_PR/sigma_PR)^2;
m_kPR=1/4*sigma_PR*C_DPR*(1+mu_PR^2)+kappa*a_PR*sigma_PR*((1/3*theta_PR+1/4*theta_tPR)*(v0_PR-lambda_0PR)-1/2*(v0_PR-lambda_0PR)^2-1/4*mu_PR^2*a0_PR^2);

% [vi,~,~,~] = fsolve(@InducesVelocityP,[5]);
% v0_PR = vi;

% 推进桨诱导速度所导出的拉力 X_PR_I = X_PR v0_PR是无量纲的
C_TPR_I = 4*v0_PR*sqrt((v0_PR-lambda_0PR)^2+B_v_PR^2+B_w_PR^2);
X_PR_I = 1/2*rho*pi*R_PR^2*(R_PR*Omega_PR)^2*C_TPR_I;

% 推进桨力，力矩在机体轴系下的表达式
F_propeller=1/2*rho*pi*R_PR^2*(R_PR*Omega_PR)^2*[C_TPR;0;0];
X_PR=F_propeller(1);


M_propeller=[0;X_PR*z_PR;-X_PR*y_PR]+1/2*rho*pi*R_PR^5*Omega_PR^2*[-m_kPR;0;0];     % *[-m_kPR;0;0];
L_PR=M_propeller(1);

F = X_PR-X_PR_I;

end


