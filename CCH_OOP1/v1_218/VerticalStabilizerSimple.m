classdef VerticalStabilizerSimple <  Helicopter
    %HORIZONTALSTABILIZERSIMPLE
    %   此处显示详细说明
    
    properties (GetAccess = protected)
        a_0;        % 升力线斜率
        delta;      % 阻力系数
        s_VS;       % 垂尾面积
        s_r;        % 方向舵面积
        xi;         % 垂尾后掠角

        x_VS;
        y_VS;
        z_VS;

    end
    
    properties (GetAccess = public)
        delta_r;    % 方向舵偏角，向右为正
        
        inteference;
        isEnable;
    end
    
    properties (Dependent)
        q_VS;
        u_VS;
        v_VS;
        w_VS;
        
        alpha_VS;   % 迎角
        beta_VS;    % 侧滑角
        k_r;        %方向舵效率因子
    end
    
    methods
        %--------------构造函数---------------
        function obj            = VerticalStabilizerSimple()
            %HORIZONTALSTABILIZERSIMPLE 构造此类的实例
            %   
        end
        %--------------Dependent变量---------------
        function q_VS           = get.q_VS(obj)
            q_VS = 1/2*obj.rho*(obj.u_VS^2+obj.v_VS^2+obj.w_VS^2);
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
        function k_r           = get.k_r(obj)
           k_r = sqrt(obj.s_r/obj.s_VS)*cos(obj.xi); 
        end
        %--------------方法--------------
        function calculate_force(obj)
            %METHOD1 此处显示有关此方法的摘要

            % 计算垂尾气流坐标系下的气动力
            Y_e     = -obj.q_VS*obj.s_VS*obj.a_0*obj.delta_r*obj.k_r;
            X_VS    = -obj.q_VS*obj.s_VS*obj.delta;
            Y_VS    = -obj.q_VS*obj.s_VS*obj.beta_VS*obj.a_0 + Y_e;
            Z_VS    = 0;
            
            % 计算机体坐标系下的气动力
            obj.X   = cos(obj.alpha_VS)*cos(obj.beta_VS) * X_VS ...
                        -cos(obj.alpha_VS)*sin(obj.beta_VS) * Y_VS ...
                        -sin(obj.alpha_VS) * Z_VS;
            obj.Y   = sin(obj.beta_VS) * X_VS ...
                        +cos(obj.beta_VS) * Y_VS;
            obj.Z   = sin(obj.alpha_VS)*cos(obj.beta_VS) * X_VS ...
                        -sin(obj.alpha_VS)*sin(obj.beta_VS) * Y_VS ...
                        +cos(obj.alpha_VS) * Z_VS;
        end
        function calculate_torque(obj)
            obj.L   = obj.y_VS*obj.Z-obj.z_VS*obj.Y;
            obj.M   = obj.z_VS*obj.X-obj.x_VS*obj.Z;
            obj.N   = obj.x_VS*obj.Y-obj.y_VS*obj.X;
        end
    end
end

