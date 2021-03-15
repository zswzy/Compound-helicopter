classdef VerticalStabilizerNASA <  Helicopter
    %HORIZONTALSTABILIZERSIMPLE
    %   此处显示详细说明
    
    properties (GetAccess = protected)
        a_0;        % 升力线斜率
        delta;      % 阻力系数
        s_VS;       % 垂尾面积
        s_r;        % 方向舵面积
        xi;         % 垂尾后掠角
        K_VS;       % 垂尾动压损失系数

        x_VS;
        y_VS;
        z_VS;
        Omega;  % 旋翼转速
        R;      % 旋翼半径

    end
    
    properties (GetAccess = public)
        delta_r;    % 方向舵偏角，向右为正
        
        inteference;
        isEnable;
    end
    
    properties (Dependent)
        A;      % 旋翼面积

        q_VS;
        u_VS;
        v_VS;
        w_VS;
        V_inf; 
        mu;
        VT;     % blade tip speed
        
        alpha_VS;   % 迎角
        beta_VS;    % 侧滑角
    end
    
    methods
        %--------------构造函数---------------
        function obj            = VerticalStabilizerNASA()
            %HORIZONTALSTABILIZERSIMPLE 构造此类的实例
            %   
        end
        %--------------Dependent变量---------------
        function A              = get.A(obj)
            A = pi*obj.R^2;
        end
        function q_VS           = get.q_VS(obj)
            q_VS = 1/2*obj.rho*(obj.u_VS^2+obj.v_VS^2+obj.w_VS^2)*obj.K_VS;
        end
        function u_VS           = get.u_VS(obj)
            u_VS = obj.u + obj.q*obj.z_VS - obj.r*obj.y_VS;
        end
        function v_VS           = get.v_VS(obj)
            v_VS = obj.v + obj.r*obj.x_VS - obj.p*obj.z_VS;
        end
        function w_VS           = get.w_VS(obj)
            w_VS = obj.w + obj.p*obj.y_VS - obj.q*obj.x_VS;
        end
        function V_inf = get.V_inf(obj)
            V_inf = sqrt(obj.u_VS^2+obj.v_VS^2+obj.w_VS^2);
        end
        function mu = get.mu(obj)
            mu = obj.V_inf/obj.VT;
        end
        function alpha_VS       = get.alpha_VS(obj)
            if obj.u_VS == 0
                alpha_VS = pi/2*sign(obj.w_VS);
            elseif obj.u_VS > 0
                alpha_VS = atan(obj.w_VS/obj.u_VS);
            else
                alpha_VS = pi + atan(obj.w_VS/obj.u_VS);
            end
        end
        function beta_VS        = get.beta_VS(obj)
            if obj.u_VS == 0
                beta_VS = pi/2*sign(obj.v_VS);
            elseif obj.u_VS > 0
                beta_VS = atan(obj.v_VS/obj.u_VS);
            else
                beta_VS = pi + atan(obj.v_VS/obj.u_VS);
            end
        end
        function VT = get.VT(obj)
            VT = obj.Omega*obj.R;
        end
        %--------------方法--------------
        function calculate_force(obj)
            %METHOD1 此处显示有关此方法的摘要
            array_mu = [0 0.1 0.21 0.31];
            array_C_Y_delta_r = 1e-5*[0   0.5     1.52    2.4];

            C_Y_delta_r = interp1(array_mu,array_C_Y_delta_r,obj.mu,'spline','extrap');
            C_YVS       = C_Y_delta_r*rad2deg(obj.delta_r);
            X_VS = 0;
            Y_VS = C_YVS*obj.rho*obj.A*obj.VT^2;
            Z_VS = 0;

            obj.X = X_VS;
            obj.Y = Y_VS;
            obj.Z = Z_VS;

        end
        function calculate_torque(obj)
            array_mu = [0 0.1 0.21 0.31];
            array_C_l_delta_r = 1e-5*[0     0.2     0.5    0.6];
            array_C_n_delta_r = 1e-5*[0     -0.5    -1.9    -3];

            C_l_delta_r = interp1(array_mu,array_C_l_delta_r,obj.mu,'spline','extrap');
            C_n_delta_r = interp1(array_mu,array_C_n_delta_r,obj.mu,'spline','extrap');

            C_lVS       = C_l_delta_r*rad2deg(obj.delta_r);
            C_nVS       = C_n_delta_r*rad2deg(obj.delta_r);

            L_VS = C_lVS*obj.rho*obj.A*obj.VT^2*obj.R;
            M_VS = 0;
            N_VS = C_nVS*obj.rho*obj.A*obj.VT^2*obj.R;

            obj.L = L_VS;
            obj.M = M_VS;
            obj.N = N_VS;

        end
    end
end

