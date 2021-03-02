classdef Propeller < Helicopter
    %PROPELLER 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties (GetAccess = protected)
        a_0;        % 升力线斜率,NACA0012
        b;          % 桨叶数 4
        c;          % 主旋翼桨叶弦长m
        direction;  % 旋转方向，后视, anticlockwise=1, clockwise=-1
        delta;      % 主旋翼桨叶阻力系数       
        h_R;        % 推进桨处于重心之上的位置
        rho;        % 空气密度
        s;          % 推进桨实度
        theta_t;    % 推进桨桨叶扭转角 rad
        x_cg;       % 推进桨处于重心之后的位置
        x_H;        % 桨毂x位置
        y_H;        % 桨毂y位置
        z_H;        % 桨毂z位置
        Omega;      % 推进桨转速rad/s
        R;          % 推进桨半径 m
        
        IsEnable;   % 1:on（default） 0:off
    end
    
    properties (GetAccess = protected, Dependent)
        A;          % 桨盘面积
    end
    
    properties (GetAccess = public)
        v_0;
        lambda_1sW = 0;
        
        theta_0;

        Omega_dot   = 0;
        
        % 桨毂系下的垂向力，分别用叶素法和动量法计算得出
        Z_h_blade_element;
        Z_h_momentum_theory;
    end
    
    properties (GetAccess = public, Dependent)
        u_H; 
        v_H;
        w_H;
        
        p_H; 
        q_H;
        r_H;
        p_dot_H;
        q_dot_H;
        r_dot_H;  

        %a_Hx;
        %a_Hy;
        %a_Hz;
        alpha_1c;
        alpha_1s;
        
        %eta_x;
        %eta_y;
        %eta_z;
        
        lambda_0;
        lambda_1s;
        lambda_1c;
        lambda_1cW;
        
        mu;
        mu_x;
        mu_y;
        mu_z;
        
        p_bar_H;
        q_bar_H;
        p_Hp;
        q_Hp;
        
        phi_W;
        Chi_W;
        
    end
    
    methods
        %----------构造函数---------------------%
        function obj = Propeller()
            %PROPELLER 构造此类的实例
            obj.IsEnable = 1;
        end
        %----------Dependent 成员函数-----------%
        function u_H            = get.u_H(obj)
            u_H = -obj.w-obj.q*obj.h_R;
        end
        function v_H            = get.v_H(obj)
            v_H = obj.v-obj.r*obj.x_cg+obj.p*obj.h_R;
        end
        function w_H            = get.w_H(obj)
            w_H = obj.u+obj.q*obj.x_cg;
        end
        function p_H            = get.p_H(obj)
            p_H = -obj.r;
        end
        function q_H            = get.q_H(obj)
            q_H = obj.q;
        end
        function r_H            = get.r_H(obj)
            r_H = obj.p;
        end
        function p_dot_H        = get.p_dot_H(obj)
            p_dot_H = -obj.r_dot;
        end
        function q_dot_H        = get.q_dot_H(obj)
            q_dot_H = obj.q_dot;
        end
        function r_dot_H        = get.r_dot_H(obj)
            r_dot_H = obj.p_dot;
        end
        function A              = get.A(obj)
            A = pi*obj.R^2;
        end
        function alpha_1c       = get.alpha_1c(obj)
           alpha_1c = obj.q_bar_H-obj.lambda_1c; 
        end
        function alpha_1s       = get.alpha_1s(obj)
           alpha_1s = obj.p_bar_H-obj.lambda_1s; 
        end
        function lambda_0       = get.lambda_0(obj)
           lambda_0 = obj.v_0/(obj.Omega*obj.R); 
        end
        function mu             = get.mu(obj)
            mu = sqrt(obj.u_H^2+obj.v_H^2)/(obj.Omega*obj.R);
        end
        function mu_x           = get.mu_x(obj)
            mu_x = obj.u_H/(obj.Omega*obj.R);
        end
        function mu_y           = get.mu_y(obj)
            mu_y = obj.v_H/(obj.Omega*obj.R);
        end
        function mu_z           = get.mu_z(obj)
            mu_z = obj.w_H/(obj.Omega*obj.R);
        end
        function p_bar_H        = get.p_bar_H(obj)
            p_bar_H = obj.p_H/obj.Omega;
        end
        function q_bar_H        = get.q_bar_H(obj)
            q_bar_H = obj.q_H/obj.Omega;
        end
        function p_Hp           = get.p_Hp(obj)
            p_Hp = obj.p_bar_H/obj.Omega^2;
        end
        function q_Hp           = get.q_Hp(obj)
            q_Hp = obj.q_bar_H/obj.Omega^2;
        end
        function phi_W          = get.phi_W(obj)
            
            if obj.mu_x == 0
                phi_W = pi/2*sign(obj.mu_y);
            elseif obj.mu_x > 0 
                phi_W = atan(obj.mu_y/obj.mu_x);
            elseif obj.mu_x < 0
                phi_W = pi + atan(obj.mu_y/obj.mu_x);
            end
        end
        function Chi_W          = get.Chi_W(obj)
           if obj.lambda_0-obj.mu_z >= 0
               Chi_W = atan(obj.mu/(obj.lambda_0-obj.mu_z));
           else
               Chi_W = pi + atan(obj.mu/(obj.lambda_0-obj.mu_z));
           end
        end
        function lambda_1cW     = get.lambda_1cW(obj)
            if obj.Chi_W <= pi/2
                lambda_1cW = obj.lambda_0*tan(obj.Chi_W/2);
            else
                lambda_1cW = obj.lambda_0*cot(obj.Chi_W/2);
            end
        end
        function lambda_1c      = get.lambda_1c(obj)
            lambda_1c = cos(obj.phi_W)*obj.lambda_1cW - sin(obj.phi_W)*obj.lambda_1sW;
        end
        function lambda_1s      = get.lambda_1s(obj)
            lambda_1s = -sin(obj.phi_W)*obj.lambda_1cW + cos(obj.phi_W)*obj.lambda_1sW;
        end
        %-----------方法----------------------%
        function calculate_force(obj)
            syms p rb_bar
            %UT_bar = @(p,rb_bar) obj.mu_x*sin(p)+obj.mu_y*cos(p)+rb_bar;
            %UP_bar = @(p,rb_bar) rb_bar*(obj.alpha_1c*cos(p)+obj.alpha_1s*sin(p))+ obj.mu_z - obj.lambda_0;
            %the = @(p,rb_bar) obj.theta_0 + rb_bar*obj.theta_t;
            %C_Zpbl = -obj.s*obj.a_0/(2*obj.b)*int( sqrt(UP_bar(p,rb_bar)^2+UT_bar(p,rb_bar)^2)*(UP_bar(p,rb_bar)*(the+atan(UP_ba(p,rb_bar)r/UT_bar(p,rb_bar)))+obj.delta/obj.a_0*UT_bar(p,rb_bar)) , 'rb_bar',0,1);
            %C_Zpbl = @(p,rb_bar) -obj.s*obj.a_0/(2*obj.b)* sqrt(UP_bar(p,rb_bar)^2+UT_bar(p,rb_bar)^2)*(UP_bar(p,rb_bar)*(the+atan(UP_bar(p,rb_bar)/UT_bar(p,rb_bar)))+obj.delta/obj.a_0*UT_bar(p,rb_bar));
            %Z_h = obj.rho*(obj.Omega*obj.R)^2*obj.A*obj.b/(2*pi)*int(C_Zpbl, 'p', -pi,pi);
            fun = @(p,rb_bar)  ( -obj.s*obj.a_0/(2*obj.b)...
                                    *sqrt((rb_bar*(obj.alpha_1c*cos(p)+obj.alpha_1s*sin(p))+ obj.mu_z - obj.lambda_0)^2+(obj.mu_x*sin(p)+obj.mu_y*cos(p)+rb_bar)^2) ...
                                    *((rb_bar*(obj.alpha_1c*cos(p)+obj.alpha_1s*sin(p))+ obj.mu_z - obj.lambda_0) ...
                                    *((obj.theta_0 + rb_bar*obj.theta_t)+atan((rb_bar*(obj.alpha_1c*cos(p)+obj.alpha_1s*sin(p))+ obj.mu_z - obj.lambda_0)/(obj.mu_x*sin(p)+obj.mu_y*cos(p)+rb_bar)))...
                                    +obj.delta/obj.a_0*(obj.mu_x*sin(p)+obj.mu_y*cos(p)+rb_bar)));
            
            Z_h = obj.rho*(obj.Omega*obj.R)^2*obj.A*obj.b/(2*pi)*doubleInt(fun,-pi,pi,0,1,16,20)
            %Z_h = obj.rho*(obj.Omega*obj.R)^2*obj.A*obj.b/(2*pi)*integral2(fun,-pi,pi,0,1 )

            %obj.X = Z_h;
        end
    end
end

