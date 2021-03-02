function P = CalculatePowerV2()
% 包含尾推的功率计算

    %% 功率计算 hpM马力 再算成W，要*746
% 1牛 = 1/4.445磅
% 1米 = 3.28英尺
% 1千克 = 2.205磅 = 1/14.5939slug
% rho : kg/m^3 = 1/14.5939/3.28^3 slug/ft^3 = 0.0019 slug/ft^3
global rho A A_PR s Omega Omega_PR R R_PR delta delta_PR Heli Rotor Propeller s_PR
f = 19.3/3.28^2;
e = 0.8;
% hpp:废阻功率，hpM旋翼诱导+型阻功率，hpPR尾推诱导+型阻功率
hpp = (rho*0.0019)*f*(Heli.V*3.28)^3/1100;
hpM1 = -Rotor.Z_h1/4.445*Rotor.v_01*3.28 /e / 550 + (rho*0.0019)*(A*3.28^2)*s*(Omega*R*3.28)^3*delta/8*(1+3*Heli.mu^2)/550;
hpM2 = -Rotor.Z_h2/4.445*Rotor.v_02*3.28 /e / 550 + (rho*0.0019)*(A*3.28^2)*s*(Omega*R*3.28)^3*delta/8*(1+3*Heli.mu^2)/550;
hpPR =  Propeller.X_PR/4.445*Propeller.v_0PR*3.28 / 550 + (rho*0.0019)*(A_PR*3.28^2)*s_PR*(Omega_PR*R_PR*3.28)^2*delta_PR/8*(1+3*Propeller.mu_PR^2)/550;

PW_P = hpp*746;
PW_MR1 = hpM1*746;
PW_MR2 = hpM2*746;
PW_PR = hpPR*746;

Power = PW_P+PW_MR1+PW_MR2+PW_PR;


Heli.PowerP = PW_P;
Rotor.Power1 = PW_MR1;
Rotor.Power2 = PW_MR2;
Propeller.Power = PW_PR;
Heli.Power = Power;
end
