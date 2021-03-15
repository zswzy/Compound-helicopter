function info_dynamics(Rotorcraft)
%	展示Rotorcraft的受力信息

VariableNames 	= {'X','Y','Z','L','M','N'};
RowNames	 		= {'LowerRotor','UpperRotor','Propeller','Fuslage','HorStab','VerStab','Gravity'};

matrix_dynamics_states = [Rotorcraft.LowerRotor.X Rotorcraft.LowerRotor.Y Rotorcraft.LowerRotor.Z Rotorcraft.LowerRotor.L Rotorcraft.LowerRotor.M Rotorcraft.LowerRotor.N ;
Rotorcraft.UpperRotor.X Rotorcraft.UpperRotor.Y Rotorcraft.UpperRotor.Z Rotorcraft.UpperRotor.L Rotorcraft.UpperRotor.M Rotorcraft.UpperRotor.N;
[Rotorcraft.Prop.X Rotorcraft.Prop.Y Rotorcraft.Prop.Z Rotorcraft.Prop.L Rotorcraft.Prop.M Rotorcraft.Prop.N]*Rotorcraft.Prop.isEnable;
[Rotorcraft.Fus.X Rotorcraft.Fus.Y Rotorcraft.Fus.Z Rotorcraft.Fus.L Rotorcraft.Fus.M Rotorcraft.Fus.N]*Rotorcraft.Fus.isEnable;
[Rotorcraft.HorStab.X Rotorcraft.HorStab.Y Rotorcraft.HorStab.Z Rotorcraft.HorStab.L Rotorcraft.HorStab.M Rotorcraft.HorStab.N]*Rotorcraft.HorStab.isEnable;
[Rotorcraft.VerStab.X Rotorcraft.VerStab.Y Rotorcraft.VerStab.Z Rotorcraft.VerStab.L Rotorcraft.VerStab.M Rotorcraft.VerStab.N]*Rotorcraft.VerStab.isEnable;
Rotorcraft.DoubleRotorHelicopter.X Rotorcraft.DoubleRotorHelicopter.Y Rotorcraft.DoubleRotorHelicopter.Z Rotorcraft.DoubleRotorHelicopter.L Rotorcraft.DoubleRotorHelicopter.M Rotorcraft.DoubleRotorHelicopter.N];

table_info_dynamics = array2table(matrix_dynamics_states, 'VariableNames', VariableNames, 'RowNames', RowNames);
disp(table_info_dynamics);

VariableNames 	= {'theta_0(deg)','theta_diff(deg)','theta_1c(deg)','theta_1s(deg)','theta(deg)','phi(deg)','v_01(lower,m/s)','v_02(upper,m/s)'};
matrix_trim = zeros(1,8);
matrix_trim(1) = rad2deg((Rotorcraft.LowerRotor.theta_0 + Rotorcraft.UpperRotor.theta_0) /2);
matrix_trim(2) = rad2deg((Rotorcraft.UpperRotor.theta_0 - Rotorcraft.LowerRotor.theta_0) /2);
matrix_trim(3) = rad2deg((Rotorcraft.UpperRotor.theta_1c + Rotorcraft.LowerRotor.theta_1c) /2);
matrix_trim(4) = rad2deg((Rotorcraft.UpperRotor.theta_1s + Rotorcraft.LowerRotor.theta_1s) /2);
matrix_trim(5) = rad2deg(Rotorcraft.DoubleRotorHelicopter.theta);
matrix_trim(6) = rad2deg(Rotorcraft.DoubleRotorHelicopter.phi);
matrix_trim(7) = Rotorcraft.LowerRotor.v_0;
matrix_trim(8) = Rotorcraft.UpperRotor.v_0;

table_info_trim = array2table(matrix_trim, 'VariableNames', VariableNames);
disp(table_info_trim);
end