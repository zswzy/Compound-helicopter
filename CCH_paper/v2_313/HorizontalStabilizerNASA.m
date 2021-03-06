classdef HorizontalStabilizerNASA <  Helicopter
    %HORIZONTALSTABILIZERSIMPLE
    %   此处显示详细说明
    
    properties (GetAccess = protected)
        a_0;        % 升力线斜率
        delta;      % 阻力系数
        s_HS;       % 平尾面积
        s_e;        % 升降舵面积
        xi;         % 平尾后掠角
        K_HS;       % 平尾动压损失系数

        x_HS;
        y_HS;
        z_HS;

        Omega;  % 旋翼转速
        R;      % 旋翼半径

    end
    
    properties (GetAccess = public)
        delta_e;    % 升降舵偏角，向下为正
        
        inteference;
        isEnable;
    end
    
    properties (Dependent)
        A;      % 旋翼面积
        
        q_HS;
        u_HS;
        v_HS;
        w_HS;
        V_inf; 
        mu;
        VT;     % blade tip speed
        
        alpha_HS;   % 迎角
        beta_HS;    % 侧滑角
        k_e;       %升降舵效率因子
    end
    
    methods
        %--------------构造函数---------------
        function obj            = HorizontalStabilizerNASA()
            %HORIZONTALSTABILIZERSIMPLE 构造此类的实例
            %   
        end
        %--------------Dependent变量---------------
        function A              = get.A(obj)
            A = pi*obj.R^2;
        end
        function q_HS           = get.q_HS(obj)
            q_HS = 1/2*obj.rho*(obj.u_HS^2+obj.v_HS^2+obj.w_HS^2)*obj.K_HS;
        end
        function u_HS           = get.u_HS(obj)
            u_HS = obj.u + obj.q*obj.z_HS - obj.r*obj.y_HS;
        end
        function v_HS           = get.v_HS(obj)
            v_HS = obj.v + obj.r*obj.x_HS - obj.p*obj.z_HS;
        end
        function w_HS           = get.w_HS(obj)
            w_HS = obj.w + obj.p*obj.y_HS - obj.q*obj.x_HS;
        end
        function V_inf = get.V_inf(obj)
            V_inf = sqrt(obj.u_HS^2+obj.v_HS^2+obj.w_HS^2);
        end
        function mu = get.mu(obj)
            mu = obj.V_inf/obj.VT;
        end
        function VT = get.VT(obj)
            VT = obj.Omega*obj.R;
        end
        function alpha_HS       = get.alpha_HS(obj)
            if obj.u_HS == 0
                alpha_HS = pi/2*sign(obj.w_HS);
            elseif obj.u_HS > 0
                alpha_HS = atan(obj.w_HS/obj.u_HS);
            else
                alpha_HS = pi + atan(obj.w_HS/obj.u_HS);
            end
        end
        function beta_HS        = get.beta_HS(obj)
            if obj.u_HS == 0
                beta_HS = pi/2*sign(obj.v_HS);
            elseif obj.u_HS > 0
                beta_HS = atan(obj.v_HS/obj.u_HS);
            else
                beta_HS = pi + atan(obj.v_HS/obj.u_HS);
            end
        end
        function k_e           = get.k_e(obj)
           k_e = sqrt(obj.s_e/obj.s_HS)*cos(obj.xi); 
        end
        %--------------方法--------------
        function calculate_force(obj)
            %METHOD1 此处显示有关此方法的摘要
            array_mu = [0 0.1 0.21 0.31];
            array_C_L_delta_e = 1e-5*[0   0.1  0.2 13];

            C_L_delta_e = interp1(array_mu,array_C_L_delta_e,obj.mu,'linear','extrap');
            C_LHS = C_L_delta_e*rad2deg(obj.delta_e);
            X_HS = 0;
            Y_HS = 0;
            Z_HS = -C_LHS*obj.rho*obj.A*obj.VT^2;

            obj.X = X_HS;
            obj.Y = Y_HS;
            obj.Z = Z_HS;
        end
        function calculate_torque(obj)
            array_mu = [0 0.1 0.21 0.31];
            array_C_m_delta_e = 1e-5*[0    -0.8    -1.1   -18.2];

            C_m_delta_e = interp1(array_mu,array_C_m_delta_e,obj.mu,'linear','extrap');
            C_mHS = C_m_delta_e*rad2deg(obj.delta_e);

            L_HS = 0;
            M_HS = C_mHS*obj.rho*obj.A*obj.VT^2*obj.R;
            N_HS = 0;

            obj.L = L_HS;
            obj.M = M_HS;
            obj.N = N_HS;
        end
        end
end

