%% helper function
% this function help to calculate the rotorcraft from our trim table

function [x_trim,Rotorcraft,power_total] = calculate_from_trim_table(table_slice)
    
    run init_build.m
    
    Rotorcraft.DoubleRotorHelicopter.U         = table_slice.U;
    Rotorcraft.DoubleRotorHelicopter.V         = 0;
    Rotorcraft.DoubleRotorHelicopter.W         = 0;
    Rotorcraft.DoubleRotorHelicopter.U_dot     = 0;
    Rotorcraft.DoubleRotorHelicopter.V_dot     = 0;
    Rotorcraft.DoubleRotorHelicopter.W_dot     = 0;
    Rotorcraft.DoubleRotorHelicopter.p         = 0;
    Rotorcraft.DoubleRotorHelicopter.q         = 0;
    Rotorcraft.DoubleRotorHelicopter.r         = 0;
    Rotorcraft.DoubleRotorHelicopter.p_dot     = 0;
    Rotorcraft.DoubleRotorHelicopter.q_dot     = 0;
    Rotorcraft.DoubleRotorHelicopter.r_dot     = 0;

    nearest_initial = table_slice{1,2:10};
    
    options                 = optimset('Display','off','TolFun',1e-15,'Maxiter',30,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
    cell_InitialStates      = {nearest_initial [0.01,0,0,0,0,0,10,10,1],[0.01,0,0,0,0,0,3,3,1], ...
                                [0.1,0,0,0,0,0,10,10,1],[0.1,0,0,0,0,0,3,3,1], ...
                                [0.2,0,0,0,0,0,10,10,1],[0.2,0,0,0,0,0,3,3,1], ...
                                [0.3,0,0,0,0,0,10,10,1],[0.3,0,0,0,0,0,3,3,1]};
    [x_trim,~,~,~,Rotorcraft,~,power_total] = trim_solve(Rotorcraft, ...
                                    @Aerodynamics_trim_full_9var, ... 
                                    cell_InitialStates, ...
                                    options, ...
                                    2, ...                  % LowerRotor.inteference
                                    2, ...                  % UpperRotor.inteference
                                    table_slice.Prop_theta_0, ...        % Prop.theta_0
                                    table_slice.Prop_isEnable, ...                  % Prop.isEnable
                                    1, ...                  % Fus.isEnable
                                    table_slice.delta_e, ...         % HorStab.delta_e
                                    1, ...                  % HorStab.isEnable
                                    table_slice.delta_r, ...         % VerStab.delta_r
                                    1, ...                  % VerStab.isEnable
                                    table_slice.theta_1c_diff, ...         % theta_1c_diff
                                    table_slice.theta_1s_diff);            % theta_1s_diff
end