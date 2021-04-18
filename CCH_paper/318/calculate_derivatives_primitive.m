function Derivatives = calculate_derivatives_primitive(Rotorcraft,x_trim)
%DERIVATIVES compute the derivatives of the force and moment, wrt states and control
%   Derivatives = derivatives(Rotorcraft)
%   Input: Rotorcraft
%           x_trim: trimmed states, 8 vars
%   Output: Derivatives: a struct, including the following section
%   <>                  :   X,Y,Z,L,M,N
%   .d<>dtheta_0        :   d()/d(theta_0)
%   .d<>dtheta_diff     :   d()/d(theta_diff)
%   .d<>dtheta_1c       :   d()/d(theta_1c)
%   .d<>dtheta_1s       :   d()/d(theta_1s)
%   .d<>dtheta          :   d()/d(theta)
%   .d<>dphi            :   d()/d(phi)
%   .d<>dU              :   d()/d(U)
%   .d<>dV              :   d()/d(V)
%   .d<>dW              :   d()/d(W)
%   .d<>dp              :   d()/d(p)
%   .d<>dq              :   d()/d(q)
%   .d<>dr              :   d()/d(r)
x_trim = x_trim(1:8);
Derivatives = struct;
epsilon     = 1e-10; % differential parameter

% calculate d()/d(theta_0)
x_trim_theta_0_left       = x_trim;
x_trim_theta_0_left(1)    = x_trim(1) - epsilon;
[~,Fnet_theta_0_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim_theta_0_left);
x_trim_theta_0_right      = x_trim;
x_trim_theta_0_right(1)   = x_trim(1) + epsilon;
[~,Fnet_theta_0_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim_theta_0_right);
d_dtheta_0 = (Fnet_theta_0_right - Fnet_theta_0_left)/(2*epsilon);
Derivatives.dXdtheta_0 = d_dtheta_0(1);
Derivatives.dYdtheta_0 = d_dtheta_0(2);
Derivatives.dZdtheta_0 = d_dtheta_0(3);
Derivatives.dLdtheta_0 = d_dtheta_0(4);
Derivatives.dMdtheta_0 = d_dtheta_0(5);
Derivatives.dNdtheta_0 = d_dtheta_0(6);

% calculate d()/d(theta_diff)
x_trim_theta_diff_left       = x_trim;
x_trim_theta_diff_left(2)    = x_trim(2) - epsilon;
[~,Fnet_theta_diff_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim_theta_diff_left);
x_trim_theta_diff_right      = x_trim;
x_trim_theta_diff_right(2)   = x_trim(2) + epsilon;
[~,Fnet_theta_diff_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim_theta_diff_right);d_dtheta_diff = (Fnet_theta_diff_right - Fnet_theta_diff_left)/(2*epsilon);
Derivatives.dXdtheta_diff = d_dtheta_diff(1);
Derivatives.dYdtheta_diff = d_dtheta_diff(2);
Derivatives.dZdtheta_diff = d_dtheta_diff(3);
Derivatives.dLdtheta_diff = d_dtheta_diff(4);
Derivatives.dMdtheta_diff = d_dtheta_diff(5);
Derivatives.dNdtheta_diff = d_dtheta_diff(6);

% calculate d()/d(theta_1c)
x_trim_theta_1c_left       = x_trim;
x_trim_theta_1c_left(3)    = x_trim(3) - epsilon;
[~,Fnet_theta_1c_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim_theta_1c_left);
x_trim_theta_1c_right      = x_trim;
x_trim_theta_1c_right(3)   = x_trim(3) + epsilon;
[~,Fnet_theta_1c_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim_theta_1c_right);
d_dtheta_1c = (Fnet_theta_1c_right - Fnet_theta_1c_left)/(2*epsilon);
Derivatives.dXdtheta_1c = d_dtheta_1c(1);
Derivatives.dYdtheta_1c = d_dtheta_1c(2);
Derivatives.dZdtheta_1c = d_dtheta_1c(3);
Derivatives.dLdtheta_1c = d_dtheta_1c(4);
Derivatives.dMdtheta_1c = d_dtheta_1c(5);
Derivatives.dNdtheta_1c = d_dtheta_1c(6);

% calculate d()/d(theta_1s)
x_trim_theta_1s_left       = x_trim;
x_trim_theta_1s_left(4)    = x_trim(4) - epsilon;
[~,Fnet_theta_1s_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim_theta_1s_left);
x_trim_theta_1s_right      = x_trim;
x_trim_theta_1s_right(4)   = x_trim(4) + epsilon;
[~,Fnet_theta_1s_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim_theta_1s_right);
d_dtheta_1s = (Fnet_theta_1s_right - Fnet_theta_1s_left)/(2*epsilon);
Derivatives.dXdtheta_1s = d_dtheta_1s(1);
Derivatives.dYdtheta_1s = d_dtheta_1s(2);
Derivatives.dZdtheta_1s = d_dtheta_1s(3);
Derivatives.dLdtheta_1s = d_dtheta_1s(4);
Derivatives.dMdtheta_1s = d_dtheta_1s(5);
Derivatives.dNdtheta_1s = d_dtheta_1s(6);

% calculate d()/d(theta_prop)
Rotorcraft.Prop.theta_0 = Rotorcraft.Prop.theta_0 - epsilon;
[~,Fnet_theta_prop_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.Prop.theta_0 = Rotorcraft.Prop.theta_0 + 2*epsilon;
[~,Fnet_theta_prop_right]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.Prop.theta_0 = Rotorcraft.Prop.theta_0 - epsilon;
d_dtheta_prop = (Fnet_theta_prop_right - Fnet_theta_prop_left)/(2*epsilon);
Derivatives.dXdtheta_prop = d_dtheta_prop(1);
Derivatives.dYdtheta_prop = d_dtheta_prop(2);
Derivatives.dZdtheta_prop = d_dtheta_prop(3);
Derivatives.dLdtheta_prop = d_dtheta_prop(4);
Derivatives.dMdtheta_prop = d_dtheta_prop(5);
Derivatives.dNdtheta_prop = d_dtheta_prop(6);

% calculate d()/d(delta_e)
Rotorcraft.HorStab.delta_e = Rotorcraft.HorStab.delta_e - epsilon;
[~,Fnet_delta_e_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.HorStab.delta_e = Rotorcraft.HorStab.delta_e + 2*epsilon;
[~,Fnet_delta_e_right]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.HorStab.delta_e = Rotorcraft.HorStab.delta_e - epsilon;
d_ddelta_e = (Fnet_delta_e_right - Fnet_delta_e_left)/(2*epsilon);
Derivatives.dXddelta_e = d_ddelta_e(1);
Derivatives.dYddelta_e = d_ddelta_e(2);
Derivatives.dZddelta_e = d_ddelta_e(3);
Derivatives.dLddelta_e = d_ddelta_e(4);
Derivatives.dMddelta_e = d_ddelta_e(5);
Derivatives.dNddelta_e = d_ddelta_e(6);

% calculate d()/d(delta_r)
Rotorcraft.VerStab.delta_r = Rotorcraft.VerStab.delta_r - epsilon;
[~,Fnet_delta_r_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.VerStab.delta_r = Rotorcraft.VerStab.delta_r + 2*epsilon;
[~,Fnet_delta_r_right]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.VerStab.delta_r = Rotorcraft.VerStab.delta_r - epsilon;
d_ddelta_r = (Fnet_delta_r_right - Fnet_delta_r_left)/(2*epsilon);
Derivatives.dXddelta_r = d_ddelta_r(1);
Derivatives.dYddelta_r = d_ddelta_r(2);
Derivatives.dZddelta_r = d_ddelta_r(3);
Derivatives.dLddelta_r = d_ddelta_r(4);
Derivatives.dMddelta_r = d_ddelta_r(5);
Derivatives.dNddelta_r = d_ddelta_r(6);

% calculate d()/d(dtheta_1c_diff)
Rotorcraft.LowerRotor.theta_1c_diff = Rotorcraft.LowerRotor.theta_1c_diff - epsilon;
Rotorcraft.UpperRotor.theta_1c_diff = Rotorcraft.UpperRotor.theta_1c_diff - epsilon;
[~,Fnet_theta_1c_diff_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.LowerRotor.theta_1c_diff = Rotorcraft.LowerRotor.theta_1c_diff + 2*epsilon;
Rotorcraft.UpperRotor.theta_1c_diff = Rotorcraft.UpperRotor.theta_1c_diff + 2*epsilon;
[~,Fnet_theta_1c_diff_right]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.LowerRotor.theta_1c_diff = Rotorcraft.LowerRotor.theta_1c_diff - epsilon;
Rotorcraft.UpperRotor.theta_1c_diff = Rotorcraft.UpperRotor.theta_1c_diff - epsilon;
d_dtheta_1c_diff = (Fnet_theta_1c_diff_right - Fnet_theta_1c_diff_left)/(2*epsilon);
Derivatives.dXdtheta_1c_diff = d_dtheta_1c_diff(1);
Derivatives.dYdtheta_1c_diff = d_dtheta_1c_diff(2);
Derivatives.dZdtheta_1c_diff = d_dtheta_1c_diff(3);
Derivatives.dLdtheta_1c_diff = d_dtheta_1c_diff(4);
Derivatives.dMdtheta_1c_diff = d_dtheta_1c_diff(5);
Derivatives.dNdtheta_1c_diff = d_dtheta_1c_diff(6);

% calculate d()/d(dtheta_1s_diff)
Rotorcraft.LowerRotor.theta_1s_diff = Rotorcraft.LowerRotor.theta_1s_diff - epsilon;
Rotorcraft.UpperRotor.theta_1s_diff = Rotorcraft.UpperRotor.theta_1s_diff - epsilon;
[~,Fnet_theta_1s_diff_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.LowerRotor.theta_1s_diff = Rotorcraft.LowerRotor.theta_1s_diff + 2*epsilon;
Rotorcraft.UpperRotor.theta_1s_diff = Rotorcraft.UpperRotor.theta_1s_diff + 2*epsilon;
[~,Fnet_theta_1s_diff_right]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.LowerRotor.theta_1s_diff = Rotorcraft.LowerRotor.theta_1s_diff - epsilon;
Rotorcraft.UpperRotor.theta_1s_diff = Rotorcraft.UpperRotor.theta_1s_diff - epsilon;
d_dtheta_1s_diff = (Fnet_theta_1s_diff_right - Fnet_theta_1s_diff_left)/(2*epsilon);
Derivatives.dXdtheta_1s_diff = d_dtheta_1s_diff(1);
Derivatives.dYdtheta_1s_diff = d_dtheta_1s_diff(2);
Derivatives.dZdtheta_1s_diff = d_dtheta_1s_diff(3);
Derivatives.dLdtheta_1s_diff = d_dtheta_1s_diff(4);
Derivatives.dMdtheta_1s_diff = d_dtheta_1s_diff(5);
Derivatives.dNdtheta_1s_diff = d_dtheta_1s_diff(6);

% calculate d()/d(theta)
x_trim_theta_left       = x_trim;
x_trim_theta_left(5)    = x_trim(5) - epsilon;
[~,Fnet_theta_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim_theta_left);
x_trim_theta_right      = x_trim;
x_trim_theta_right(5)   = x_trim(5) + epsilon;
[~,Fnet_theta_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim_theta_right);
d_dtheta = (Fnet_theta_right - Fnet_theta_left)/(2*epsilon);
Derivatives.dXdtheta = d_dtheta(1);
Derivatives.dYdtheta = d_dtheta(2);
Derivatives.dZdtheta = d_dtheta(3);
Derivatives.dLdtheta = d_dtheta(4);
Derivatives.dMdtheta = d_dtheta(5);
Derivatives.dNdtheta = d_dtheta(6);

% calculate d()/d(phi)
x_trim_phi_left       = x_trim;
x_trim_phi_left(6)    = x_trim(6) - epsilon;
[~,Fnet_phi_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim_phi_left);
x_trim_phi_right      = x_trim;
x_trim_phi_right(6)   = x_trim(6) + epsilon;
[~,Fnet_phi_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim_phi_right);
d_dphi = (Fnet_phi_right - Fnet_phi_left)/(2*epsilon);
Derivatives.dXdphi = d_dphi(1);
Derivatives.dYdphi = d_dphi(2);
Derivatives.dZdphi = d_dphi(3);
Derivatives.dLdphi = d_dphi(4);
Derivatives.dMdphi = d_dphi(5);
Derivatives.dNdphi = d_dphi(6);

% calculate d()/d(U)
Rotorcraft.DoubleRotorHelicopter.U   = Rotorcraft.DoubleRotorHelicopter.U - epsilon;
[~,Fnet_U_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.U  = Rotorcraft.DoubleRotorHelicopter.U + 2*epsilon;
[~,Fnet_U_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.U   = Rotorcraft.DoubleRotorHelicopter.U - epsilon;
d_dU = (Fnet_U_right - Fnet_U_left)/(2*epsilon);
Derivatives.dXdU = d_dU(1);
Derivatives.dYdU = d_dU(2);
Derivatives.dZdU = d_dU(3);
Derivatives.dLdU = d_dU(4);
Derivatives.dMdU = d_dU(5);
Derivatives.dNdU = d_dU(6);

% calculate d()/d(V)
Rotorcraft.DoubleRotorHelicopter.V   = Rotorcraft.DoubleRotorHelicopter.V - epsilon;
[~,Fnet_V_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.V  = Rotorcraft.DoubleRotorHelicopter.V + 2*epsilon;
[~,Fnet_V_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.V   = Rotorcraft.DoubleRotorHelicopter.V- epsilon;
d_dV = (Fnet_V_right - Fnet_V_left)/(2*epsilon);
Derivatives.dXdV = d_dV(1);
Derivatives.dYdV = d_dV(2);
Derivatives.dZdV = d_dV(3);
Derivatives.dLdV = d_dV(4);
Derivatives.dMdV = d_dV(5);
Derivatives.dNdV = d_dV(6);

% calculate d()/d(W)
Rotorcraft.DoubleRotorHelicopter.W   = Rotorcraft.DoubleRotorHelicopter.W - epsilon;
[~,Fnet_W_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.W  = Rotorcraft.DoubleRotorHelicopter.W + 2*epsilon;
[~,Fnet_W_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.W   = Rotorcraft.DoubleRotorHelicopter.W - epsilon;
d_dW = (Fnet_W_right - Fnet_W_left)/(2*epsilon);
Derivatives.dXdW = d_dW(1);
Derivatives.dYdW = d_dW(2);
Derivatives.dZdW = d_dW(3);
Derivatives.dLdW = d_dW(4);
Derivatives.dMdW = d_dW(5);
Derivatives.dNdW = d_dW(6);

% calculate d()/d(u)
scale_factor = cos(Rotorcraft.DoubleRotorHelicopter.theta)*cos(Rotorcraft.DoubleRotorHelicopter.psi);
Rotorcraft.DoubleRotorHelicopter.U   = Rotorcraft.DoubleRotorHelicopter.U - epsilon ;
[~,Fnet_u_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.U  = Rotorcraft.DoubleRotorHelicopter.U + 2*epsilon;
[~,Fnet_u_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.U   = Rotorcraft.DoubleRotorHelicopter.U - epsilon;
d_du = (Fnet_u_right - Fnet_u_left)/(2*epsilon/scale_factor);
Derivatives.dXdu = d_du(1);
Derivatives.dYdu = d_du(2);
Derivatives.dZdu = d_du(3);
Derivatives.dLdu = d_du(4);
Derivatives.dMdu = d_du(5);
Derivatives.dNdu = d_du(6);

% calculate d()/d(v)
scale_factor = sin(Rotorcraft.DoubleRotorHelicopter.theta)*sin(Rotorcraft.DoubleRotorHelicopter.phi)*sin(Rotorcraft.DoubleRotorHelicopter.psi)+cos(Rotorcraft.DoubleRotorHelicopter.phi)*cos(Rotorcraft.DoubleRotorHelicopter.psi);
Rotorcraft.DoubleRotorHelicopter.V   = Rotorcraft.DoubleRotorHelicopter.V - epsilon;
[~,Fnet_v_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.V  = Rotorcraft.DoubleRotorHelicopter.V + 2*epsilon;
[~,Fnet_v_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.V   = Rotorcraft.DoubleRotorHelicopter.V- epsilon;
d_dv = (Fnet_v_right - Fnet_v_left)/(2*epsilon/scale_factor);
Derivatives.dXdv = d_dv(1);
Derivatives.dYdv = d_dv(2);
Derivatives.dZdv = d_dv(3);
Derivatives.dLdv = d_dv(4);
Derivatives.dMdv = d_dv(5);
Derivatives.dNdv = d_dv(6);

% calculate d()/d(w)
scale_factor = cos(Rotorcraft.DoubleRotorHelicopter.theta)*cos(Rotorcraft.DoubleRotorHelicopter.phi);
Rotorcraft.DoubleRotorHelicopter.W   = Rotorcraft.DoubleRotorHelicopter.W - epsilon;
[~,Fnet_w_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.W  = Rotorcraft.DoubleRotorHelicopter.W + 2*epsilon;
[~,Fnet_w_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.W   = Rotorcraft.DoubleRotorHelicopter.W - epsilon;
d_dw = (Fnet_w_right - Fnet_w_left)/(2*epsilon);
Derivatives.dXdw = d_dw(1);
Derivatives.dYdw = d_dw(2);
Derivatives.dZdw = d_dw(3);
Derivatives.dLdw = d_dw(4);
Derivatives.dMdw = d_dw(5);
Derivatives.dNdw = d_dw(6);

% calculate d()/d(p)
Rotorcraft.DoubleRotorHelicopter.p   = Rotorcraft.DoubleRotorHelicopter.p - epsilon;
[~,Fnet_p_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.p  = Rotorcraft.DoubleRotorHelicopter.p + 2*epsilon;
[~,Fnet_p_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.p   = Rotorcraft.DoubleRotorHelicopter.p - epsilon;
d_dp = (Fnet_p_right - Fnet_p_left)/(2*epsilon);
Derivatives.dXdp = d_dp(1);
Derivatives.dYdp = d_dp(2);
Derivatives.dZdp = d_dp(3);
Derivatives.dLdp = d_dp(4);
Derivatives.dMdp = d_dp(5);
Derivatives.dNdp = d_dp(6);

% calculate d()/d(q)
Rotorcraft.DoubleRotorHelicopter.q   = Rotorcraft.DoubleRotorHelicopter.q - epsilon;
[~,Fnet_q_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.q  = Rotorcraft.DoubleRotorHelicopter.q + 2*epsilon;
[~,Fnet_q_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.q   = Rotorcraft.DoubleRotorHelicopter.q - epsilon;
d_dq = (Fnet_q_right - Fnet_q_left)/(2*epsilon);
Derivatives.dXdq = d_dq(1);
Derivatives.dYdq = d_dq(2);
Derivatives.dZdq = d_dq(3);
Derivatives.dLdq = d_dq(4);
Derivatives.dMdq = d_dq(5);
Derivatives.dNdq = d_dq(6);

% calculate d()/d(r)
Rotorcraft.DoubleRotorHelicopter.r   = Rotorcraft.DoubleRotorHelicopter.r - epsilon;
[~,Fnet_r_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.r  = Rotorcraft.DoubleRotorHelicopter.r + 2*epsilon;
[~,Fnet_r_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim);
Rotorcraft.DoubleRotorHelicopter.r   = Rotorcraft.DoubleRotorHelicopter.r - epsilon;
d_dr = (Fnet_r_right - Fnet_r_left)/(2*epsilon);
Derivatives.dXdr = d_dr(1);
Derivatives.dYdr = d_dr(2);
Derivatives.dZdr = d_dr(3);
Derivatives.dLdr = d_dr(4);
Derivatives.dMdr = d_dr(5);
Derivatives.dNdr = d_dr(6);

end


