function Derivatives = derivatives(Rotorcraft,x_trim)
%DERIVATIVES compute the derivatives of the force and moment, wrt states and control
%   Derivatives = derivatives(Rotorcraft)
%   Input: Rotorcraft
%           x_trim: trimmed states, 8 vars
%   Output: Derivatives: a struct, including the following section
%   .dXdtheta   :   d(X)/d(theta)
%   .dMdtheta   :   d(M)/d(theta)
%   .dXdU       :   d(X)/d(U)

Derivatives = struct;
epsilon     = 1e-10;

%   calculate d()/d(theta)
Rotorcraft.DoubleRotorHelicopter.theta  = Rotorcraft.DoubleRotorHelicopter.theta - epsilon;
x_trim_theta_left   = x_trim;
x_trim_theta_left(5)    = x_trim(5) - epsilon;
[~,Fnet_theta_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim_theta_left);

Rotorcraft.DoubleRotorHelicopter.theta  = Rotorcraft.DoubleRotorHelicopter.theta + 2*epsilon;
x_trim_theta_right  = x_trim;
x_trim_theta_right(5)   = x_trim(5) + epsilon;
[~,Fnet_theta_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim_theta_right);

d_dtheta = (Fnet_theta_right-Fnet_theta_left)/(2*epsilon);
Derivatives.dXdtheta = d_dtheta(1);
Derivatives.dYdtheta = d_dtheta(2);
Derivatives.dZdtheta = d_dtheta(3);
Derivatives.dLdtheta = d_dtheta(4);
Derivatives.dMdtheta = d_dtheta(5);
Derivatives.dNdtheta = d_dtheta(6);
Rotorcraft.DoubleRotorHelicopter.theta  = Rotorcraft.DoubleRotorHelicopter.theta - epsilon;

%   calculate d()/d(U)
Rotorcraft.DoubleRotorHelicopter.U   = Rotorcraft.DoubleRotorHelicopter.U - epsilon;
[~,Fnet_U_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);

Rotorcraft.DoubleRotorHelicopter.U  = Rotorcraft.DoubleRotorHelicopter.U + 2*epsilon;
[~,Fnet_U_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim);

d_dU = (Fnet_U_right-Fnet_U_left)/(2*epsilon);
Derivatives.dXdU = d_dU(1);
Derivatives.dYdU = d_dU(2);
Derivatives.dZdU = d_dU(3);
Derivatives.dLdU = d_dU(4);
Derivatives.dMdU = d_dU(5);
Derivatives.dNdU = d_dU(6);
Rotorcraft.DoubleRotorHelicopter.U   = Rotorcraft.DoubleRotorHelicopter.U - epsilon;

%   calculate d()/d(W)
Rotorcraft.DoubleRotorHelicopter.W   = Rotorcraft.DoubleRotorHelicopter.W - epsilon;
[~,Fnet_W_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);

Rotorcraft.DoubleRotorHelicopter.W  = Rotorcraft.DoubleRotorHelicopter.W + 2*epsilon;
[~,Fnet_W_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim);

d_dW = (Fnet_W_right-Fnet_W_left)/(2*epsilon);
Derivatives.dXdW = d_dW(1);
Derivatives.dYdW = d_dW(2);
Derivatives.dZdW = d_dW(3);
Derivatives.dLdW = d_dW(4);
Derivatives.dMdW = d_dW(5);
Derivatives.dNdW = d_dW(6);
Rotorcraft.DoubleRotorHelicopter.W   = Rotorcraft.DoubleRotorHelicopter.W - epsilon;

%   calculate d()/d(q)
Rotorcraft.DoubleRotorHelicopter.q   = Rotorcraft.DoubleRotorHelicopter.q - epsilon;
[~,Fnet_q_left]     = Aerodynamics_full_8var(Rotorcraft, x_trim);

Rotorcraft.DoubleRotorHelicopter.q  = Rotorcraft.DoubleRotorHelicopter.q + 2*epsilon;
[~,Fnet_q_right]    = Aerodynamics_full_8var(Rotorcraft, x_trim);

d_dq = (Fnet_q_right-Fnet_q_left)/(2*epsilon);
Derivatives.dXdq = d_dq(1);
Derivatives.dYdq = d_dq(2);
Derivatives.dZdq = d_dq(3);
Derivatives.dLdq = d_dq(4);
Derivatives.dMdq = d_dq(5);
Derivatives.dNdq = d_dq(6);
Rotorcraft.DoubleRotorHelicopter.q   = Rotorcraft.DoubleRotorHelicopter.q - epsilon;

end


