function power_total = calculate_power(Rotorcraft)
%CALCULATE_POWER 此处显示有关此函数的摘要
%   此处显示详细说明
power_total = Rotorcraft.LowerRotor.Power_total ...
                + Rotorcraft.UpperRotor.Power_total ...
                + Rotorcraft.Prop.Power_resist ...
                + Rotorcraft.Prop.Power_induced * Rotorcraft.Prop.isEnable;
end

