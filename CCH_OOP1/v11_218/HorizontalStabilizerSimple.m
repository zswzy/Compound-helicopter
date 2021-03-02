classdef HorizontalStabilizerSimple <  Helicopter
    %HORIZONTALSTABILIZERSIMPLE
    %   此处显示详细说明
    
    properties (GetAccess = protected)
        a_0;        % 升力线斜率
        delta;      % 阻力系数
        s_HS;       % 平尾面积
        s_e;        % 升降舵面积
        xi;         % 平尾后掠角

        x_HS;
        y_HS;
        z_HS;

    end
    
    properties (GetAccess = public)
        delta_e;    % 升降舵偏角，向下为正
        
        inteference;
        isEnable;
    end
    
    properties (Dependent)
        q_HS;
        u_HS;
        v_HS;
        w_HS;
        
        alpha_HS;   % 迎角
        beta_HS;    % 侧滑角
        k_e;       %升降舵效率因子
    end
    
    methods
        %--------------构造函数---------------
        function obj            = HorizontalStabilizerSimple()
            %HORIZONTALSTABILIZERSIMPLE 构造此类的实例
            %   
        end
        %--------------Dependent变量---------------
        function q_HS           = get.q_HS(obj)
            q_HS = 1/2*obj.rho*(obj.u_HS^2+obj.v_HS^2+obj.w_HS^2);
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

            % 计算平尾坐标系下的气动力
            Z_e     = -obj.q_HS*obj.s_HS*obj.a_0*obj.delta_e*obj.k_e;
            X_HS    = -obj.q_HS*obj.s_HS*obj.delta;
            Y_HS    = 0;
            Z_HS    = - obj.q_HS*obj.s_HS*obj.alpha_HS*obj.a_0 + Z_e;
            
            % 计算机体坐标系下的气动力
            obj.X   = cos(obj.alpha_HS)*cos(obj.beta_HS) * X_HS ...
                        -cos(obj.alpha_HS)*sin(obj.beta_HS) * Y_HS ...
                        -sin(obj.alpha_HS) * Z_HS;
            obj.Y   = sin(obj.beta_HS) * X_HS ...
                        +cos(obj.beta_HS) * Y_HS;
            obj.Z   = sin(obj.alpha_HS)*cos(obj.beta_HS) * X_HS ...
                        -sin(obj.alpha_HS)*sin(obj.beta_HS) * Y_HS ...
                        +cos(obj.alpha_HS) * Z_HS;
        end
        function calculate_torque(obj)
            obj.L   = obj.y_HS*obj.Z-obj.z_HS*obj.Y;
            obj.M   = obj.z_HS*obj.X-obj.x_HS*obj.Z;
            obj.N   = obj.x_HS*obj.Y-obj.y_HS*obj.X;
        end
    end
end

