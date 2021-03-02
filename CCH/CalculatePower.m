function P = CalculatePower()


    %% 功率计算 hpM马力 再算成W，要*746
% 1牛 = 1/4.445磅
% 1米 = 3.28英尺
% 1千克 = 2.205磅 = 1/14.5939slug
% rho : kg/m^3 = 1/14.5939/3.28^3 slug/ft^3 = 0.0019 slug/ft^3
global rho A s Omega R delta Heli Rotor
f = 19.3/3.28^2;

hpM1 = -Rotor.Z_h1/4.445*Rotor.v_01*3.28 / 550 +(rho*0.0019)*f*(Heli.V*3.28)^3/1100 + (rho*0.0019)*(A*3.28^2)*s*(Omega*R*3.28)^3*delta/8*(1+3*Heli.mu^2)/550;
hpM2 = -Rotor.Z_h2/4.445*Rotor.v_02*3.28 / 550 +(rho*0.0019)*f*(Heli.V*3.28)^3/1100 + (rho*0.0019)*(A*3.28^2)*s*(Omega*R*3.28)^3*delta/8*(1+3*Heli.mu^2)/550;
%hpT =  X_PR_I/4.445*v0_PR*3.28 / 550 + (rho*0.0019)*(A*3.28^2)*sigma_PR*(Omega_PR*R_PR*3.28)^2*c_d/8*(1+3*mu^2)/550;

%hpM1 = (T_W1_I/4.445)^2/(1000*(rho*0.0019)*(A*3.28^2)*(V*3.28)*(0.8))+(rho*0.0019)*f*(V*3.28)^3/1100 + (rho*0.0019)*(A*3.28^2)*sigma_M*(Omega*R*3.28)^2*c_d/8*(1+3*mu^2)/550;
%hpM2 = (T_W2_I/4.445)^2/(1000*(rho*0.0019)*(A*3.28^2)*(V*3.28)*(0.8))+(rho*0.0019)*f*(V*3.28)^3/1100 + (rho*0.0019)*(A*3.28^2)*sigma_M*(Omega*R*3.28)^2*c_d/8*(1+3*mu^2)/550;
%hpT = (X_PR_I/4.445)^2/(1000*(rho*0.0019)*pi*(R_PR*3.28)^2*(V*3.28)) + (rho*0.0019)*(A*3.28^2)*sigma_PR*(Omega_PR*R_PR*3.28)^2*c_d/8*(1+3*mu^2)/550;

PW_MR1 = hpM1*746;
PW_MR2 = hpM2*746;
PW_PR = 0;
Power = PW_MR1+PW_MR2+PW_PR;
Rotor.Power = Power;
Rotor.Power1 = PW_MR1;
Rotor.Power2 = PW_MR2;
end
