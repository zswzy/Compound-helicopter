classdef FuselageLynx < Helicopter
    %FUSELAGE 机身
    %   
    properties (GetAccess = protected)
        s_F;    % 机身有效面积 19.3?
        x_F;
        y_F;
        z_F;
        R;      % 旋翼半径
        A;      % 旋翼面积
    end
    properties (GetAccess = public)
        isEnable;
    end
    properties (Dependent)
        u_F;
        v_F;
        w_F;
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
        function obj = FuselageLynx()
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
            % C_LF
            C_X0F = -0.0217;
            C_X1F = 0.0602;
            C_YF = -0.1721;
            C_ZF = -0.0817;
            
            X_F         = 1/2*obj.rho*(obj.u_F^2+obj.v_F^2+obj.w_F^2)*obj.A*(C_X0F+C_X1F*obj.alpha_F^2);
            Y_F         = 1/2*obj.rho*(obj.u_F^2+obj.v_F^2+obj.w_F^2)*obj.A*(C_YF*obj.beta_F);
            Z_F         = 1/2*obj.rho*(obj.u_F^2+obj.v_F^2+obj.w_F^2)*obj.A*(C_ZF*obj.alpha_F);
          
            obj.X       = X_F;
            obj.Y       = Y_F;
            obj.Z       = Z_F;
        end
        
        function calculate_torque(obj)
            % calculate_torque
            
            C_LF = 0;
            C_MF = 0.0179;
            C_NF = 0.0179;
            
            L_F = 0;
            M_F = 1/2*obj.rho*(obj.u_F^2+obj.v_F^2+obj.w_F^2)*obj.A*2*obj.R*(C_MF*obj.alpha_F);
            N_F = 1/2*obj.rho*(obj.u_F^2+obj.v_F^2+obj.w_F^2)*obj.A*2*obj.R*(C_NF*obj.beta_F);

            obj.L = L_F + obj.y_F*obj.Z-obj.z_F*obj.Y;
            obj.M = M_F + obj.z_F*obj.X-obj.x_F*obj.Z;
            obj.N = N_F + obj.x_F*obj.Y-obj.y_F*obj.X;
        end
    end
end

