%Propeller Force and Moment NUAA Caoyan


X_PR = X_PR_fitted(theta_0PR);
Y_PR = 0;
Z_PR = 0;
PropellerForce = [X_PR;Y_PR;Z_PR];

L_PR = L_PR_fitted(theta_0PR);
M_PR = z_PR*X_PR;
N_PR = -y_PR*X_PR;
PropellerMoment = [L_PR;M_PR;N_PR];

