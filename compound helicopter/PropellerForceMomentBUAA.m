%Propeller model

k_1PR = 1.229 + 0.671*Chi_1 - 1.172*Chi_1^2 + 0.351*Chi_1^3;
k_2PR = 1.229 + 0.671*Chi_2 - 1.172*Chi_2^2 + 0.351*Chi_2^3;
w_iPR = k_1PR*v_1 + k_2PR*v_2;

u_PR = u*sqrt(K_PR) + z_PR*q - y_PR*r;
v_PR = v*sqrt(K_PR) + x_PR*r - z_PR*p;
w_PR = w*sqrt(K_PR) + y_PR*p - x_PR*q + w_iPR;

u_bar_PR = u_PR/(Omega_PR*R_PR);
v_bar_PR = v_PR/(Omega_PR*R_PR);
w_bar_PR = w_PR/(Omega_PR*R_PR);

mu_PR = sqrt(v_bar_PR ^2 + w_bar_PR ^2);
lambda_PR = -u_bar_PR;

a_0PR = kappa*gamma_PR/2*(1/4*(1+mu_PR^2)*theta_0PR + (1/5+1/6*mu_PR^2)*theta_tPR - 1/3*(v_0PR-lambda_PR));
C_TPR = a_PR*sigma_PR*((1/3+1/2*mu_PR^2)*theta_0PR + 1/4*(1+mu_PR^2)*theta_tPR - 1/2*(v_0PR-lambda_PR));

m_kPR = 1/4*sigma_PR*c_dPR*(1+mu_PR^2) + kappa*a_PR*sigma_PR...
    *((1/3*theta_0PR + 1/4*theta_tPR)*(v_0PR-lambda_PR) - 1/2*(v_0PR-lambda_PR)^2 - 1/4*mu_PR^2*a_0PR^2);

X_PR = 1/2*rho*pi*R_PR^2*(Omega_PR*R_PR)^2*C_TPR;
Y_PR = 0;
Z_PR = 0;
PropellerForce = [X_PR;Y_PR;Z_PR]

L_PR = 1/2*rho*pi*R_PR^3*(Omega_PR*R_PR)^2*(-m_kPR);
M_PR = z_PR*X_PR;
N_PR = -y_PR*X_PR;
PropellerMoment = [L_PR;M_PR;N_PR]

    