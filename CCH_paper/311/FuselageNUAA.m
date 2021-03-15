classdef FuselageNUAA < Helicopter
    %FUSELAGE 机身
    %   
    properties (GetAccess = protected)
        s_F;    % 机身有效面积 19.3?
        x_F;
        y_F;
        z_F;
        R;      % 旋翼半径
        A;      % 旋翼面积
        Omega;
    end
    properties (GetAccess = public)
        isEnable;
    end
    properties (Dependent)
        u_F;
        v_F;
        w_F;
        V_inf;
        u_dot_F;
        v_dot_F;
        w_dot_F;
        
        p_F;
        q_F;
        r_F;
        p_dot_F;
        q_dot_F;
        r_dot_F;
        
        alpha_F; % 迎角
        beta_F;  % 侧滑角
        
        Power_total;
    end
    
    methods
        function obj = FuselageNUAA()
            %FUSELAGE 构造此类的实例
        end
        function u_F = get.u_F(obj)
            u_F = obj.u;
        end
        function v_F = get.v_F(obj)
            v_F = obj.v;
        end
        function w_F = get.w_F(obj)
            w_F = obj.w;
        end
        function V_inf = get.V_inf(obj)
            V_inf = sqrt(obj.u_F^2+obj.v_F^2+obj.w_F^2);
        end
        function u_dot_F = get.u_dot_F(obj)
            u_dot_F = obj.u_dot;
        end
        function v_dot_F = get.v_dot_F(obj)
            v_dot_F = obj.v_dot;
        end
        function w_dot_F = get.w_dot_F(obj)
            w_dot_F = obj.w_dot;
        end
        function p_F = get.p_F(obj)
            p_F = obj.p;
        end
        function q_F = get.q_F(obj)
            q_F = obj.q;
        end
        function r_F = get.r_F(obj)
            r_F = obj.r;
        end
        function p_dot_F = get.p_dot_F(obj)
            p_dot_F = obj.p_dot;
        end
        function q_dot_F = get.q_dot_F(obj)
            q_dot_F = obj.q_dot;
        end
        function r_dot_F = get.r_dot_F(obj)
            r_dot_F = obj.r_dot;
        end
        function alpha_F    = get.alpha_F(obj)
            if obj.u_F == 0
                alpha_F = pi/2*sign(obj.w_F);
            elseif obj.u_F > 0
                alpha_F = atan(obj.w_F/obj.u_F);
            else
                alpha_F = pi + atan(obj.w_F/obj.u_F);
            end
        end
        function beta_F    = get.beta_F(obj)
            if obj.u_F == 0
                beta_F = pi/2*sign(obj.v_F);
            elseif obj.u_F > 0
                beta_F = atan(obj.v_F/obj.u_F);
            else
                beta_F = pi + atan(obj.v_F/obj.u_F);
            end
        end
        function Power_total = get.Power_total(obj)
            Power_total = obj.rho*obj.s_F*(sqrt(obj.U^2+obj.V^2+obj.W^2))^3;
        end
        
        function calculate_force(obj)
            % calculate_force
            C_XF = 90.0555*sin(obj.alpha_F)^2 - 41.5604*cos(obj.alpha_F) + 2.94684*cos(4*obj.beta_F)-103.14*cos(2*obj.beta_F) ...
                    -0.53535e-6*obj.beta_F^4+160.2049;
            C_YF = 29.3616*sin(obj.alpha_F) + 43.468*sin(2*obj.alpha_F) - 81.8924*sin(obj.alpha_F)^2 - 84.1469*cos(obj.alpha_F) ...
                    -0.821406e-1*obj.beta_F - 3.00102*sin(4*obj.beta_F) + 0.323477*obj.beta_F^2+85.3496;
            C_ZF = -35.3999*sin(obj.beta_F) - 71.8019*sin(2*obj.beta_F) + 8.04823*sin(4*obj.beta_F);
            
            X_F         = - 1/2*obj.rho*obj.V_inf^2*obj.s_F*C_XF;
            Y_F         = 1/2*obj.rho*obj.V_inf^2*obj.s_F*C_YF;
            Z_F         = 1/2*obj.rho*obj.V_inf^2*obj.s_F*C_ZF;
            
            X = cos(obj.alpha_F)*cos(obj.beta_F)*X_F + cos(obj.alpha_F)*sin(obj.beta_F)*Y_F - sin(obj.alpha_F)*Z_F;
            Y = sin(obj.beta_F)*X_F - cos(obj.beta_F)*Y_F;
            Z = sin(obj.alpha_F)*cos(obj.beta_F)*X_F + sin(obj.alpha_F)*sin(obj.beta_F)*Y_F + cos(obj.alpha_F)*Z_F;
            
            obj.X       = X;
            obj.Y       = Y;
            obj.Z       = Z;
        end
        
        function calculate_torque(obj)
            % calculate_torque
            
            C_MF = 2.37925*obj.alpha_F + 728.026*sin(2*obj.alpha_F) + 426.76*sin(obj.alpha_F)^2 + 348.072*cos(obj.alpha_F) - 510.581*cos(obj.beta_F)^3 + 56.111;
            
            if (deg2rad(25) < abs(obj.beta_F) && abs(obj.beta_F) <= pi/2)
                C_LF = -614.797*sin(obj.beta_F) - obj.beta_F/abs(obj.beta_F)*(-47.7213*cos(4*obj.beta_F) - 290.504*cos(obj.beta_F)^3 + 735.507*cos(4*obj.beta_F) - 669.266);
            elseif (deg2rad(10) < abs(obj.beta_F) && abs(obj.beta_F) <= deg2rad(25))
                C_LF = - obj.beta_F/abs(obj.beta_F)*(455.707*cos(obj.beta_F)^4 - 428.639);
            else
                C_LF = 0;
            end
            
            if (deg2rad(20) < abs(obj.beta_F) && abs(obj.beta_F) <= pi/2)
                C_NF = -220.0*sin(2*obj.beta_F) - obj.beta_F/abs(obj.beta_F)*(671.0*cos(obj.beta_F)^4-429.0);
            else
                C_NF = 278.133*sin(2*obj.beta_F) - 422.644*sin(4*obj.beta_F) - 1.83172 ;
            end
            
            L_F = 1/2*obj.rho*obj.V_inf^2*obj.s_F*2*obj.R*C_LF;
            M_F = 1/2*obj.rho*obj.V_inf^2*obj.s_F*2*obj.R*C_MF;
            N_F = 1/2*obj.rho*obj.V_inf^2*obj.s_F*2*obj.R*C_NF;

            
            L = cos(obj.alpha_F)*cos(obj.beta_F)*L_F + cos(obj.alpha_F)*sin(obj.beta_F)*M_F - sin(obj.alpha_F)*N_F;
            M = sin(obj.beta_F)*L_F - cos(obj.beta_F)*M_F;
            N = sin(obj.alpha_F)*cos(obj.beta_F)*L_F + sin(obj.alpha_F)*sin(obj.beta_F)*M_F + cos(obj.alpha_F)*N_F;
            
            obj.L = L;
            obj.M = M;
            obj.N = N;
        end
    end
end

