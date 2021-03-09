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

end