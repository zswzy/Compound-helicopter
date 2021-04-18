classdef RotorFixed < Helicopter
    %RotorFixed 旋翼类 继承Helicopter
    %   2.9 p15 p16公式修正
    %   2.10 p26 挥舞公式修正
    
    properties (GetAccess = protected)
        a_0;        % 主旋翼升力线斜率,NACA0012
        b;          % 旋翼桨叶数
        c;          % 主旋翼桨叶弦长m
        delta;      % 主旋翼桨叶阻力系数
        e;          % 无量纲等效铰偏置量
        e_oswald;   % 奥斯瓦尔德效率因子 (0.8)
        gamma_s;    % 主旋翼桨毂纵向安装角
        h_R;        % 主旋翼处于重心之上的位置
        m_b;        % 桨叶质量
        omega_n;    % 主旋翼一阶挥舞固有频率 
        s;          % 主旋翼实度
        theta_t;    % 主旋翼桨叶扭转角 rad
        x_cg;       % 主旋翼处于重心之后的位置
        x_H;        % 桨毂x位置
        y_H;        % 桨毂y位置
        z_H;        % 桨毂z位置
        z_diff;     % 上下旋翼间距 m
        I_beta;     % 主旋翼挥舞惯性矩 kg m^2
        K_beta;     % K_beta 弹簧刚度  Nm/rad
        M_beta;     % 主旋翼对挥舞铰的质量静矩
        Omega;      % 主旋翼转速rad/s
        R;          % 主旋翼半径 m
    end
    
    properties (GetAccess = protected, Dependent)
        A;          % 桨盘面积
        I_bar_beta;
    end
    
    
    properties (GetAccess = public)
        v_0; %被干扰后的诱导速度
        v_i; %固有诱导速度
        
        direction;  % 旋转方向，俯视, anticlockwise=1, clockwise=-1
        lambda_1sW = 0;
        
        theta_0;
        theta_1c;
        theta_1s;
        theta_1c_diff;
        theta_1s_diff;
                 
        beta_0;
        beta_1c;
        beta_1s;
        beta_0p     = 0;
        beta_1cp    = 0;
        beta_1sp    = 0;
        beta_0pp    = 0; 
        beta_1cpp   = 0; 
        beta_1spp   = 0;
        
        inteference;% 启动干扰 0：无干扰 1：直接相加干扰 2：干扰因子干扰
        
        Omega_dot   = 0;
        
        % 拉力，分别用叶素法和动量法计算得出
        T_blade_element;
        T_momentum_theory;
        
        X_H;
        Y_H;
        Z_H;
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

        a_Hx;
        a_Hy;
        a_Hz;
        alpha_1c;
        alpha_1s;
        
        delta_1;
        delta_2;
        
        eta_x;
        eta_y;
        eta_z;
        
        epsilon_1c;
        epsilon_1s;
        
        gamma_1c;
        gamma_1s;
        
        lambda_i;
        lambda_0;
        lambda_1s;
        lambda_1c;
        lambda_1cW;
        lambda_beta;
        
        m_bar_b;
        M_bar_beta;
        mu;
        mu_x;
        mu_y;
        mu_z;
        
        n_beta;
        
        p_bar_H;
        q_bar_H;
        p_Hp;
        q_Hp;
        
        phi_W;
        Chi_W;
        
        T; % 总拉力
        
        Power_total;
        Power_induced;
        Power_resist;
        
    end
     

    methods
        %----------构造函数---------------------%
        function obj = RotorFixed(rotate_direction)
            %ROTOR 构造此类的实例
            if strcmp(rotate_direction, 'anticlockwise')
                obj.direction = 1;
            elseif strcmp(rotate_direction, 'clockwise')
                obj.direction = -1;
            end
        end
        %----------Dependent 成员函数-----------%
        function u_H            = get.u_H(obj)
            u_H = (obj.u-obj.q*obj.h_R)*cos(obj.gamma_s)+...
                (obj.w+obj.q*obj.x_cg)*sin(obj.gamma_s);
        end
        function v_H            = get.v_H(obj)
            v_H = obj.v+obj.p*obj.h_R-obj.r*obj.x_cg;
        end
        function w_H            = get.w_H(obj)
            w_H = (obj.w+obj.q*obj.x_cg)*cos(obj.gamma_s)-...
                (obj.u-obj.q*obj.h_R)*sin(obj.gamma_s);
        end
        function p_H            = get.p_H(obj)
           p_H = obj.p*cos(obj.gamma_s) + obj.r*sin(obj.gamma_s); 
        end
        function q_H            = get.q_H(obj)
           q_H = obj.q; 
        end
        function r_H            = get.r_H(obj)
           r_H = obj.r*cos(obj.gamma_s) - obj.p*sin(obj.gamma_s); 
        end
        function p_dot_H        = get.p_dot_H(obj)
           p_dot_H = obj.p_dot*cos(obj.gamma_s) + obj.r_dot*sin(obj.gamma_s); 
        end
        function q_dot_H        = get.q_dot_H(obj)
           q_dot_H = obj.q_dot; 
        end
        function r_dot_H        = get.r_dot_H(obj)
           r_dot_H = obj.r_dot*cos(obj.gamma_s) - obj.p_dot*sin(obj.gamma_s); 
        end
        function A              = get.A(obj)
            A = pi*obj.R^2;
        end    
        function I_bar_beta     =get.I_bar_beta(obj)
            I_bar_beta = obj.I_beta/(obj.rho*pi*obj.R^5);
        end
        function a_Hx           = get.a_Hx(obj)
            a_Hx = (obj.u_dot-(obj.q_dot+obj.p*obj.r)...
                +(obj.q^2+obj.r^2))*cos(obj.gamma_s)...
                +(obj.w_dot+(obj.p^2+obj.q^2)...
                +(obj.q_dot-obj.p*obj.r)*sin(obj.gamma_s));
        end        
        function a_Hy           = get.a_Hy(obj)
            a_Hy = obj.v_dot+(obj.p_dot-obj.q*obj.r)-(obj.r_dot+obj.p*obj.q);
        end
        function a_Hz           = get.a_Hz(obj)
            a_Hz = (obj.w_dot+(obj.p^2+obj.q^2)+...
                (obj.q_dot-obj.p*obj.r))*cos(obj.gamma_s)-...
                (obj.u_dot-(obj.q_dot+obj.p*obj.r)+...
                (obj.q^2+obj.r^2))*sin(obj.gamma_s);
        end
        function alpha_1c       = get.alpha_1c(obj)
            alpha_1c = obj.q_bar_H - obj.lambda_1c - obj.beta_1cp - obj.beta_1s;
        end
        function alpha_1s       = get.alpha_1s(obj)
            alpha_1s = obj.p_bar_H - obj.lambda_1s - obj.beta_1sp + obj.beta_1c; 
        end
        function delta_1        = get.delta_1(obj)
            if obj.inteference == 0
                delta_1 = 0;
            elseif obj.inteference == 1
                delta_1 = 1;
            elseif obj.inteference == 2
                delta_1_list = [1.35	1.31	1.2     1.05	0.9     0.8	0.7     0.57	0.44]';
                mu_list = [0	0.0128	0.041	0.0808	0.1205	0.15	0.191	0.2282	0.2705]';
                %delta_1Fitted = fit(mu_list, delta_1_list, 'linearinterp');
                if obj.mu < 0.2705
                    %delta_1 = delta_1Fitted(obj.mu);
                    delta_1 = interp1(mu_list,delta_1_list,obj.mu,'spline');
                else
                    delta_1 = 0.44*exp(-(obj.mu-0.2705)/(0.3962-0.2705));
                end
            end
        end
        function delta_2        = get.delta_2(obj)
            if obj.inteference == 0 || obj.inteference == 1
                delta_2 = 0;
            elseif obj.inteference == 2
                delta_2_list = [0.66	0.64	0.58	0.53	0.45	0.4	0.33	0.21	0.11]';
                mu_list = [0	0.0128	0.041	0.0808	0.1205	0.15	0.191	0.2282	0.2705]';
                %delta_2Fitted = fit(mu_list, delta_2_list, 'linearinterp');
                if obj.mu < 0.2705
                    %delta_2 = delta_2Fitted(obj.mu);
                    delta_2 = interp1(mu_list,delta_2_list,obj.mu,'spline');
                else
                    delta_2 = 0.11*exp(-(obj.mu-0.2705)/(0.3077-0.2705));
                end
            end
        end
        function eta_x          = get.eta_x(obj)
            eta_x = obj.a_Hx/(obj.Omega^2*obj.R);
        end
        function eta_y          = get.eta_y(obj)
            eta_y = obj.a_Hy/(obj.Omega^2*obj.R);
        end
        function eta_z          = get.eta_z(obj)
            eta_z = obj.a_Hz/(obj.Omega^2*obj.R);
        end
        function epsilon_1c     = get.epsilon_1c(obj)
            epsilon_1c = obj.beta_1cp+obj.beta_1s-obj.q_bar_H;
        end
        function epsilon_1s     = get.epsilon_1s(obj)
            epsilon_1s = obj.beta_1sp-obj.beta_1c-obj.p_bar_H;
        end
        function gamma_1c       = get.gamma_1c(obj)
            gamma_1c = obj.beta_1cpp ...
                        +obj.Omega_dot/(obj.Omega^2)*obj.beta_1s ...
                        +2*obj.beta_1sp ...
                        -obj.beta_1c ...
                        -obj.q_dot_H/obj.Omega^2 ...
                        -obj.p_bar_H;
        end
        function gamma_1s       = get.gamma_1s(obj)
            gamma_1s = obj.beta_1spp ...
                        -obj.Omega_dot/(obj.Omega^2)*obj.beta_1c ...
                        -2*obj.beta_1cp ...
                        -obj.beta_1s ...
                        -obj.p_dot_H/obj.Omega^2 ...
                        +obj.q_bar_H;
        end
        function m_bar_b        = get.m_bar_b(obj)
            m_bar_b = obj.m_b/(obj.rho*pi*obj.R^3);
        end
        function M_bar_beta     = get.M_bar_beta(obj)
            M_bar_beta = obj.M_beta/(obj.rho*pi*obj.R^4);
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
        function lambda_i       = get.lambda_i(obj)
            lambda_i = obj.v_i/(obj.Omega*obj.R);  
        end
        function lambda_0       = get.lambda_0(obj)
            lambda_0 = obj.v_0/(obj.Omega*obj.R);  
        end
        function lambda_beta    = get.lambda_beta(obj)
            lambda_beta = sqrt(1+obj.K_beta/(obj.I_beta*obj.Omega^2));
        end
        function n_beta         = get.n_beta(obj)
            n_beta = obj.rho*obj.c*obj.a_0*obj.R^4/(8*obj.I_beta);
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
        function T              = get.T(obj)
            T = sqrt(obj.X_H^2+obj.Y_H^2+obj.Z_H^2);
        end
        function Power_total    = get.Power_total(obj)
            Power_total = obj.Power_induced + obj.Power_resist;
        end
        function Power_induced    = get.Power_induced(obj)
            Power_induced = abs(obj.T)*obj.v_i / obj.e_oswald;
        end
        function Power_resist    = get.Power_resist(obj)
            Power_resist = obj.rho*obj.A*obj.s*(obj.Omega*obj.R)^3*obj.delta/8*(1+3*(sqrt(obj.U^2+obj.V^2+obj.W^2)/(obj.Omega*obj.R))^2);
        end
           
        %-----------方法----------------------%          
        function calculate_flapping_angle(obj)
            % 计算三个挥舞角,四桨叶
            hM0     = zeros(4,1);
            h_0     = obj.M_beta*obj.R/obj.I_beta*obj.eta_z ...
                        +obj.n_beta*(2/3*(1-obj.e^3)*(2*(obj.mu_z-obj.lambda_0)+((2*obj.theta_1c-obj.lambda_1c+obj.q_bar_H)*obj.mu_y+(2*obj.theta_1s-obj.lambda_1s+obj.p_bar_H)*obj.mu_x)) ...
                        +(1-obj.e^2)*(1+obj.e^2+obj.mu^2)*obj.theta_0+( 4/5*(1-obj.e^5)+2/3*(1-obj.e^3)*obj.mu^2 )*obj.theta_t); %fixed
            h_1c    = 2*(obj.p_bar_H+obj.q_Hp/2) ...
                        +obj.n_beta*((1-obj.e^2)*((1+obj.e^2+1/2*obj.mu^2+obj.mu_y^2)*obj.theta_1c+obj.mu_x*obj.mu_y*obj.theta_1s)+8/3*(1-obj.e^3)*obj.mu_y*obj.theta_0 ...
                        +2*(1-obj.e^4)*obj.mu_y*obj.theta_t+2*(1-obj.e^2)*(obj.mu_z-obj.lambda_0)*obj.mu_y-(1-obj.e^4)*(obj.lambda_1c-obj.q_bar_H)); %fixed
            h_1s    = -2*(obj.q_bar_H-obj.p_Hp/2) ...
                        +obj.n_beta*((1-obj.e^2)*((1+obj.e^2+1/2*obj.mu^2+obj.mu_x^2)*obj.theta_1s+obj.mu_x*obj.mu_y*obj.theta_1c)+8/3*(1-obj.e^3)*obj.mu_x*obj.theta_0 ...
                        +2*(1-obj.e^4)*obj.mu_x*obj.theta_t+2*(1-obj.e^2)*(obj.mu_z-obj.lambda_0)*obj.mu_x-(1-obj.e^4)*(obj.lambda_1s-obj.p_bar_H)); %fixed
            hM0(1)  = h_0;
            hM0(3)  = h_1c;
            hM0(4)  = h_1s;
            
            DM0 = zeros(4,4);
            DM0(1,1) = obj.lambda_beta^2;
            DM0(1,3) = obj.M_beta*obj.R/(2*obj.I_beta)*obj.eta_x;
            DM0(1,4) = -obj.M_beta*obj.R/(2*obj.I_beta)*obj.eta_y;
            
            DM0(2,2) = obj.lambda_beta^2;
            
            DM0(3,1) = +obj.M_beta*obj.R/obj.I_beta*obj.eta_x+4/3*obj.mu_x*(1-obj.e^3)*obj.n_beta;
            DM0(3,3) = obj.lambda_beta^2-1+obj.mu_x*obj.mu_y*(1-obj.e^2)*obj.n_beta;
            DM0(3,4) = 1/2*obj.n_beta*(1-obj.e^2)*(2+2*obj.e^2+obj.mu_x^2-obj.mu_y^2);
            
            DM0(4,1) = -obj.M_beta*obj.R/obj.I_beta*obj.eta_y-4/3*obj.mu_y*(1-obj.e^3)*obj.n_beta;
            DM0(4,3) = -1/2*obj.n_beta*(1-obj.e^2)*(2+2*obj.e^2-obj.mu_x^2+obj.mu_y^2);
            DM0(4,4) = obj.lambda_beta^2-1-obj.mu_x*obj.mu_y*(1-obj.e^2)*obj.n_beta;
            
            flapping_angle = DM0 \ hM0;
            obj.beta_0  = flapping_angle(1);
            obj.beta_1c = flapping_angle(3);
            obj.beta_1s = flapping_angle(4);
            
%            disp(['beta_0: ', num2str(obj.beta_0)])
%            disp(['beta_1c: ', num2str(obj.beta_1c), ' beta_1s: ', num2str(obj.beta_1s)])
%             disp(['lambda_1c: ', num2str(obj.lambda_1c), ' lambda_1s: ', num2str(obj.lambda_1s)])
%             
%             disp(hM0)
%             disp(DM0)
            
        end
        function calculate_flapping_angle_3blades(obj)
            % 计算三个挥舞角，三桨叶，经验证可用
            hM0     = zeros(3,1);
            h_0     = obj.M_beta*obj.R/obj.I_beta*obj.eta_z ...
                        +obj.n_beta*(2/3*(1-obj.e^3)*(2*(obj.mu_z-obj.lambda_0)+((2*obj.theta_1c-obj.lambda_1c+obj.q_bar_H)*obj.mu_y+(2*obj.theta_1s-obj.lambda_1s+obj.p_bar_H)*obj.mu_x)) ...
                        +(1-obj.e^2)*(1+obj.e^2+obj.mu^2)*obj.theta_0+( 4/5*(1-obj.e^5)+2/3*(1-obj.e^3)*obj.mu^2 )*obj.theta_t); %fixed
            h_1c    = 2*(obj.p_bar_H+obj.q_Hp/2) ...
                        +obj.n_beta*((1-obj.e^2)*((1+obj.e^2+1/2*obj.mu^2+obj.mu_y^2)*obj.theta_1c+obj.mu_x*obj.mu_y*obj.theta_1s)+8/3*(1-obj.e^3)*obj.mu_y*obj.theta_0 ...
                        +2*(1-obj.e^4)*obj.mu_y*obj.theta_t+2*(1-obj.e^2)*(obj.mu_z-obj.lambda_0)*obj.mu_y-(1-obj.e^4)*(obj.lambda_1c-obj.q_bar_H)); %fixed
            h_1s    = -2*(obj.q_bar_H-obj.p_Hp/2) ...
                        +obj.n_beta*((1-obj.e^2)*((1+obj.e^2+1/2*obj.mu^2+obj.mu_x^2)*obj.theta_1s+obj.mu_x*obj.mu_y*obj.theta_1c)+8/3*(1-obj.e^3)*obj.mu_x*obj.theta_0 ...
                        +2*(1-obj.e^4)*obj.mu_x*obj.theta_t+2*(1-obj.e^2)*(obj.mu_z-obj.lambda_0)*obj.mu_x-(1-obj.e^4)*(obj.lambda_1s-obj.p_bar_H)); %fixed
            hM0(1)  = h_0;
            hM0(2)  = h_1c;
            hM0(3)  = h_1s;
            
            DM0 = zeros(3,3);
            DM0(1,1) = obj.lambda_beta^2;
            DM0(1,2) = obj.M_beta*obj.R/(2*obj.I_beta)*obj.eta_x;
            DM0(1,3) = -obj.M_beta*obj.R/(2*obj.I_beta)*obj.eta_y;
            
            DM0(2,1) = obj.M_beta*obj.R/obj.I_beta*obj.eta_x+4/3*obj.mu_x*(1-obj.e^3)*obj.n_beta;
            DM0(2,2) = obj.lambda_beta^2-1+obj.mu_x*obj.mu_y*(1-obj.e^2)*obj.n_beta;
            DM0(2,3) = 1/2*obj.n_beta*(1-obj.e^2)*(2+2*obj.e^2+obj.mu_x^2-obj.mu_y^2);
            
            DM0(3,1) = -obj.M_beta*obj.R/obj.I_beta*obj.eta_y-4/3*obj.mu_y*(1-obj.e^3)*obj.n_beta;
            DM0(3,2) = -1/2*obj.n_beta*(1-obj.e^2)*(2+2*obj.e^2-obj.mu_x^2+obj.mu_y^2);
            DM0(3,3) = obj.lambda_beta^2-1-obj.mu_x*obj.mu_y*(1-obj.e^2)*obj.n_beta;
            
            flapping_angle = DM0 \ hM0;
            obj.beta_0  = flapping_angle(1);
            obj.beta_1c = flapping_angle(2);
            obj.beta_1s = flapping_angle(3);
            
%            disp(['beta_0: ', num2str(obj.beta_0)])
%            disp(['beta_1c: ', num2str(obj.beta_1c), ' beta_1s: ', num2str(obj.beta_1s)])
%             disp(['lambda_1c: ', num2str(obj.lambda_1c), ' lambda_1s: ', num2str(obj.lambda_1s)])
%             
%             disp(hM0)
%             disp(DM0)
        end
        function calculate_force(obj)
            % 计算旋翼受力
            
            % 计算桨叶气动力系数
            C_ZA0   = -1/3*obj.beta_0p*(1-obj.e^3) ...
                        +1/2*(1-obj.e^2)*(obj.mu_z-obj.lambda_0)...
                        +1/4*(1-obj.e^2)*(obj.mu_x*(obj.alpha_1s-obj.beta_1c) ...
                        +obj.mu_y*(obj.alpha_1c+obj.beta_1s)) ...
                        +1/2*(1-obj.e^2)*(obj.theta_1c*obj.mu_y+obj.theta_1s*obj.mu_x) ...
                        +(1/3*(1-obj.e^3)+1/2*(1-obj.e)*obj.mu^2)*obj.theta_0 ...
                        +(1/4*(1-obj.e^4)+1/4*obj.mu^2*(1-obj.e^2))*obj.theta_t; % fixed
            C_ZA1c  = 1/3*obj.alpha_1c*(1-obj.e^3) ...
                        +obj.mu_y*(1-obj.e)*(obj.mu_z-obj.lambda_0) ...
                        -1/2*(1-obj.e^2)*(obj.beta_0p*obj.mu_y+obj.beta_0*obj.mu_x) ...
                        -1/4*(1-obj.e)*obj.beta_1s*(obj.mu_x^2-obj.mu_y^2) ...
                        +1/2*(1-obj.e)*(obj.theta_1s-obj.beta_1c)*obj.mu_x*obj.mu_y ...
                        +(1/3*(1-obj.e^3)+(1-obj.e)*(1/4*obj.mu^2+1/2*obj.mu_y^2))*obj.theta_1c ...
                        +obj.mu_y*(1-obj.e^2)*obj.theta_0 ...
                        +2/3*obj.mu_y*(1-obj.e^3)*obj.theta_t; %fixed
            C_ZA1s  = 1/3*obj.alpha_1s*(1-obj.e^3) ...
                        +obj.mu_x*(1-obj.e)*(obj.mu_z-obj.lambda_0) ...
                        -1/2*(1-obj.e^2)*(obj.beta_0p*obj.mu_x-obj.beta_0*obj.mu_y) ...
                        -1/4*(1-obj.e)*obj.beta_1c*(obj.mu_x^2-obj.mu_y^2) ...
                        +1/2*(1-obj.e)*(obj.theta_1c+obj.beta_1s)*obj.mu_x*obj.mu_y ...
                        +(1/3*(1-obj.e^3)+(1-obj.e)*(1/4*obj.mu^2+1/2*obj.mu_x^2))*obj.theta_1s ...
                        +obj.mu_x*(1-obj.e^2)*obj.theta_0 ...
                        +2/3*obj.mu_x*(1-obj.e^3)*obj.theta_t; % fixed
            C_YA1c  = obj.theta_1c*(1/3*obj.beta_0p*(1-obj.e^3) ...
                            -1/2*(1-obj.e^2)*(obj.mu_z-obj.lambda_0) ...
                            -1/8*(1-obj.e^2)*((obj.alpha_1s-3*obj.beta_1c)*obj.mu_x + (3*obj.alpha_1c+obj.beta_1s)*obj.mu_y) ...
                            +1/2*(1-obj.e)*obj.beta_0*obj.mu_x*obj.mu_y) ...
                        +obj.theta_1s*(-1/8*(1-obj.e^2)*((obj.alpha_1c-obj.beta_1s)*obj.mu_x + (obj.alpha_1s+obj.beta_1c)*obj.mu_y) ...
                            + 1/4*(1-obj.e)*obj.beta_0*(obj.mu_x^2-obj.mu_y^2)) ...
                        +obj.theta_0*(-1/3*(1-obj.e^3)*obj.alpha_1c ...
                            +1/2*(1-obj.e^2)*(obj.beta_0p*obj.mu_y+obj.beta_0*obj.mu_x) ...
                            -obj.mu_y*(1-obj.e)*(obj.mu_z-obj.lambda_0) ...
                            +1/4*(1-obj.e)*obj.beta_1s*(obj.mu_x^2-obj.mu_y^2) ...
                            +1/2*(1-obj.e)*obj.beta_1c*obj.mu_x*obj.mu_y) ...
                        +obj.theta_t*(-obj.alpha_1c*(1/4-1/4*obj.e^4) ...
                            +1/3*(1-obj.e^3)*(obj.beta_0p*obj.mu_y+obj.beta_0*obj.mu_x) ...
                            -1/2*obj.mu_y*(obj.mu_z-obj.lambda_0)*(1-obj.e^2) ...
                            +1/8*obj.beta_1s*(obj.mu_x^2-obj.mu_y^2)*(1-obj.e^2) ...
                            +1/4*obj.beta_1c*obj.mu_x*obj.mu_y*(1-obj.e^2)) ... %fixed
                        +obj.alpha_1c*(2/3*(1-obj.e^3)*obj.beta_0p ...
                            -(obj.mu_z-obj.lambda_0)*(1-obj.e^2) ...
                            +3/4*obj.beta_1c*obj.mu_x*(1-obj.e^2) ...
                            -1/4*obj.beta_1s*obj.mu_y*(1-obj.e^2))...
                        +1/4*obj.alpha_1s*(1-obj.e^2)*(obj.beta_1s*obj.mu_x-obj.beta_1c*obj.mu_y) ...
                        +obj.beta_0*obj.mu_x*(-obj.beta_0p*(1-obj.e^2) ...
                            +2*(1-obj.e)*(obj.mu_z-obj.lambda_0) ...
                            -3/2*obj.beta_1c*obj.mu_x*(1-obj.e) ...
                            +obj.beta_1s*obj.mu_y*(1-obj.e)) ...
                        -1/2*obj.beta_0*obj.beta_1c*obj.mu_y^2*(1-obj.e) ...
                        +obj.delta/obj.a_0*(1-obj.e^2)*obj.mu_y;
            C_YA1s  = obj.theta_1c*(-1/8*(1-obj.e^2)*((obj.alpha_1c-obj.beta_1s)*obj.mu_x + (obj.alpha_1s+obj.beta_1c)*obj.mu_y) ...
                            + 1/4*(1-obj.e)*obj.beta_0*(obj.mu_x^2-obj.mu_y^2)) ...
                        +obj.theta_1s*(1/3*obj.beta_0p*(1-obj.e^3)-1/2*(1-obj.e^2)*(obj.mu_z-obj.lambda_0) ...
                            -1/8*(1-obj.e^2)*((3*obj.alpha_1s-obj.beta_1c)*obj.mu_x + (obj.alpha_1c+3*obj.beta_1s)*obj.mu_y) ...
                            -1/2*(1-obj.e)*obj.beta_0*obj.mu_x*obj.mu_y) ... 
                        +obj.theta_0*(-1/3*(1-obj.e^3)*obj.alpha_1s ...
                            +1/2*(1-obj.e^2)*(obj.beta_0p*obj.mu_x-obj.beta_0*obj.mu_y) ...
                            -obj.mu_x*(1-obj.e)*(obj.mu_z-obj.lambda_0) ...
                            +1/4*(1-obj.e)*obj.beta_1c*(obj.mu_x^2-obj.mu_y^2) ...
                            -1/2*(1-obj.e)*obj.beta_1s*obj.mu_x*obj.mu_y) ...
                        +obj.theta_t*(-obj.alpha_1s*(1/4-1/4*obj.e^4) ...
                            +1/3*(1-obj.e^3)*(obj.beta_0p*obj.mu_x-obj.beta_0*obj.mu_y) ...
                            -1/2*obj.mu_x*(obj.mu_z-obj.lambda_0)*(1-obj.e^2) ...
                            +1/8*obj.beta_1c*(obj.mu_x^2-obj.mu_y^2)*(1-obj.e^2) ...
                            -1/4*obj.beta_1s*obj.mu_x*obj.mu_y*(1-obj.e^2)) ... % fixed
                        +obj.alpha_1s*(2/3*(1-obj.e^3)*obj.beta_0p ...
                            -(obj.mu_z-obj.lambda_0)*(1-obj.e^2) ...
                            -3/4*obj.beta_1s*obj.mu_y*(1-obj.e^2) ...
                            +1/4*obj.beta_1c*obj.mu_x*(1-obj.e^2)) ...
                        +1/4*obj.alpha_1c*(1-obj.e^2)*(obj.beta_1s*obj.mu_x-obj.beta_1c*obj.mu_y) ...
                        +obj.beta_0*obj.mu_y*(obj.beta_0p*(1-obj.e^2) ...
                            -2*(1-obj.e)*(obj.mu_z-obj.lambda_0) ...
                            -3/2*obj.beta_1s*obj.mu_y*(1-obj.e) ...
                            +obj.beta_1c*obj.mu_x*(1-obj.e)) ...
                        -1/2*obj.beta_0*obj.beta_1s*obj.mu_x^2*(1-obj.e) ...
                        +obj.delta/obj.a_0*(1-obj.e^2)*obj.mu_x;
            
            % 计算桨叶离心力系数的一阶谐波分量
            C_XI0   = obj.eta_z*obj.beta_0*obj.m_bar_b ...
                        +obj.M_bar_beta*(1+obj.beta_0p^2+1/2*(obj.epsilon_1s^2+obj.epsilon_1c^2));
            C_XI1c  = (obj.eta_x+obj.eta_z*obj.beta_1c)*obj.m_bar_b ...
                        +2*obj.beta_0p*obj.epsilon_1c*obj.M_bar_beta;
            C_XI1s  = -(obj.eta_y-obj.eta_z*obj.beta_1s)*obj.m_bar_b ...
                        +2*obj.beta_0p*obj.epsilon_1s*obj.M_bar_beta;
            C_YI1c  = obj.eta_y*obj.m_bar_b ...
                        -obj.M_bar_beta*(obj.beta_0*obj.epsilon_1c+obj.beta_0p*(obj.beta_1c-obj.p_bar_H));
            C_YI1s  = obj.eta_x*obj.m_bar_b ...
                        -obj.M_bar_beta*(obj.beta_0*obj.epsilon_1s+obj.beta_0p*(obj.beta_1s+obj.q_bar_H));
            C_ZI0   = -(obj.eta_z+1/2*(obj.eta_y*obj.beta_1s-obj.eta_x*obj.beta_1c))*obj.m_bar_b ...
                        +obj.M_bar_beta*(obj.beta_0+obj.beta_0pp);
            C_ZI1c  = obj.eta_x*obj.beta_0*obj.m_bar_b ...
                        +obj.M_bar_beta*(obj.gamma_1c+obj.beta_1c-obj.p_bar_H);
            C_ZI1s  = -obj.eta_y*obj.beta_0*obj.m_bar_b ...
                        +obj.M_bar_beta*(obj.gamma_1s+obj.beta_1s+obj.q_bar_H);
            
            % 计算桨叶力系数的一阶谐波分量
            C_X0    = C_XI0; 
            C_X1c   = C_XI1c; 
            C_X1s   = C_XI1s;
            C_Y1c   = obj.s*obj.a_0/(2*obj.b)*C_YA1c+C_YI1c; 
            C_Y1s   = obj.s*obj.a_0/(2*obj.b)*C_YA1s+C_YI1s;
            C_Z0    = -obj.s*obj.a_0/(2*obj.b)*C_ZA0+C_ZI0; 
            C_Z1c   = -obj.s*obj.a_0/(2*obj.b)*C_ZA1c+C_ZI1c; 
            C_Z1s   = -obj.s*obj.a_0/(2*obj.b)*C_ZA1s+C_ZI1s;

            % 计算桨毂力系数
            C_Xh    = -obj.b/2*(C_X1c+C_Y1s+obj.beta_0*C_Z1c+obj.beta_1c*C_Z0);
            C_Yh    = obj.b/2*(C_X1s-C_Y1c+obj.beta_0*C_Z1s+obj.beta_1s*C_Z0);
            C_Zh    = obj.b/2*(2*C_Z0-2*obj.beta_0*C_X0-obj.beta_1c*C_X1c-obj.beta_1s*C_X1s);
            C_Ti    = 2*obj.lambda_i*sqrt(obj.mu^2+(obj.mu_z-obj.lambda_0)^2); %使用诱导速度算出的垂向力系数
            % 计算在桨毂系下的力
            X_h     = obj.rho*(obj.Omega*obj.R)^2*obj.A*C_Xh;
            Y_h     = obj.rho*(obj.Omega*obj.R)^2*obj.A*C_Yh;
            Z_h     = obj.rho*(obj.Omega*obj.R)^2*obj.A*C_Zh;
            T_i     = obj.rho*(obj.Omega*obj.R)^2*obj.A*C_Ti; %使用诱导速度算出的垂向力
            
            Y_h = Y_h*obj.direction; %镜像转换
            
            obj.X_H = X_h;
            obj.Y_H = Y_h;
            obj.Z_H = Z_h;
            
            obj.T_blade_element       = sqrt(X_h^2+Y_h^2+Z_h^2);
            obj.T_momentum_theory     = T_i;
            
            % 计算在体轴系下的力
            obj.X   = X_h*cos(obj.gamma_s) - Z_h*sin(obj.gamma_s); 
            obj.Y   = Y_h;
            obj.Z   = X_h*sin(obj.gamma_s) + Z_h*cos(obj.gamma_s);
            
%            disp('------------------------------------')
            
%             disp(['C_X1s: ',num2str(C_X1s)])
%             disp(['C_Y1c: ',num2str(C_Y1c)])
%             disp(['C_Z1s: ',num2str(C_Z1s)])
%             disp(['C_ZI1s: ',num2str(C_ZI1s)])
%             disp(['C_ZA1s: ',num2str(C_ZA1s)])
%             disp(['C_X1c(C_XI1c): ', num2str(C_X1c), ' C_X1s(C_XI1s): ', num2str(C_X1s)])
%             disp(['C_Y1c: ', num2str(C_Y1c), ' C_Y1s: ', num2str(C_Y1s)])
%             disp(['C_Z1c: ', num2str(C_Z1c), ' C_Z1s: ', num2str(C_Z1s)])
%             disp(['C_YA1c: ', num2str(C_YA1c), ' C_YA1s: ', num2str(C_YA1s)])
%             disp(['C_ZA1c: ', num2str(C_ZA1c), ' C_ZA1s: ', num2str(C_ZA1s)])
%             disp(['C_YI1c: ', num2str(C_YI1c), ' C_YI1s: ', num2str(C_YI1s)])
%             disp(['C_ZI1c: ', num2str(C_ZI1c), ' C_ZI1s: ', num2str(C_ZI1s)])
%             disp(['C_Z0: ', num2str(C_Z0)])
%             disp(['alpha_1c: ', num2str(obj.alpha_1c), ' alpha_1s: ', num2str(obj.alpha_1s)])
%            disp(['X:', num2str(obj.X), ' Y:', num2str(obj.Y),' Z:', num2str(obj.Z)])
            
        end
        function calculate_torque(obj)
            % 计算旋翼力矩
            
            % 计算气动力矩系数
            C_QA0   = obj.theta_1c*(-1/8*(1-obj.e^4)*obj.alpha_1c ...
                            +1/6*(1-obj.e^3)*(obj.beta_0p*obj.mu_y+obj.beta_0*obj.mu_x) ...
                            +1/16*(1-obj.e^2)*obj.beta_1s*(obj.mu_x^2-obj.mu_y^2) ...
                            -1/4*obj.mu_y*(1-obj.e^2)*(obj.mu_z-obj.lambda_0) ...
                            +1/8*(1-obj.e^2)*obj.beta_1c*obj.mu_x*obj.mu_y) ...
                        +obj.theta_1s*(-1/8*(1-obj.e^4)*obj.alpha_1s ...
                            +1/6*(1-obj.e^3)*(obj.beta_0p*obj.mu_x-obj.beta_0*obj.mu_y) ...
                            +1/16*(1-obj.e^2)*obj.beta_1c*(obj.mu_x^2-obj.mu_y^2) ...
                            -1/4*obj.mu_x*(1-obj.e^2)*(obj.mu_z-obj.lambda_0) ...
                            -1/8*(1-obj.e^2)*obj.beta_1s*obj.mu_x*obj.mu_y) ...
                        +obj.theta_0*(1/4*(1-obj.e^4)*obj.beta_0p ...
                            -1/3*(1-obj.e^3)*(obj.mu_z-obj.lambda_0) ...
                            -1/6*(1-obj.e^3)*((obj.alpha_1s-obj.beta_1c)*obj.mu_x+(obj.alpha_1c+obj.beta_1s)*obj.mu_y)) ...
                        +obj.theta_t*(1/5*(1-obj.e^5)*obj.beta_0p ...
                            -1/4*(1-obj.e^4)*(obj.mu_z-obj.lambda_0 ...
                                +1/2*obj.mu_x*(obj.alpha_1s-obj.beta_1c) ...
                                +1/2*obj.mu_y*(obj.alpha_1c+obj.beta_1s))) ... %fixed
                        -1/8*(1-obj.e^4)*(obj.alpha_1c^2+obj.alpha_1s^2+2*obj.beta_0p^2) ...
                        +(obj.mu_z-obj.lambda_0)*(2/3*obj.beta_0p*(1-obj.e^3) ...
                            -1/2*(1-obj.e^2)*((obj.mu_z-obj.lambda_0)-(obj.beta_1c*obj.mu_x-obj.beta_1s*obj.mu_y))) ...
                        +1/3*(1-obj.e^3)*(obj.beta_0*(obj.alpha_1c*obj.mu_x-obj.alpha_1s*obj.mu_y) ...
                            +obj.beta_0p*(obj.beta_1s*obj.mu_y-obj.beta_1c*obj.mu_x)) ...
                        +(1-obj.e^2)*(-1/4*obj.beta_0^2*obj.mu^2 ...
                            +1/4*obj.beta_1s*obj.beta_1c*obj.mu_x*obj.mu_y ...
                            -1/16*obj.mu^2*(obj.beta_1s^2+obj.beta_1c^2) ...
                            -1/8*(obj.beta_1c^2*obj.mu_x^2+obj.beta_1s^2*obj.mu_y^2)) ...
                        +1/4*(1-obj.e^2)*(1+obj.e^2+obj.mu^2)*obj.delta/obj.a_0;

            C_QA1c  = obj.theta_1c*(1/4*obj.beta_0p*(1-obj.e^4) ...
                            +(1-obj.e^3)*(-1/3*(obj.mu_z-obj.lambda_0) ...
                                -1/12*(obj.alpha_1s*obj.mu_x+obj.beta_1s*obj.mu_y) ...
                                +1/4*(obj.beta_1c*obj.mu_x-obj.alpha_1c*obj.mu_y)) ...
                            +1/4*(1-obj.e^2)*obj.beta_0*obj.mu_x*obj.mu_y) ...
                        +obj.theta_1s*(1/8*(1-obj.e^2)*obj.beta_0*(obj.mu_x^2-obj.mu_y^2) ...
                            +1/12*(1-obj.e^3)*((obj.beta_1s-obj.alpha_1c)*obj.mu_x-(obj.alpha_1s+obj.beta_1c)*obj.mu_y)) ...
                        +obj.theta_0*(-1/4*(1-obj.e^4)*obj.alpha_1c+1/3*(1-obj.e^3)*(obj.beta_0p*obj.mu_y+obj.beta_0*obj.mu_x) ...
                            +1/2*(1-obj.e^2)*(1/4*obj.beta_1s*(obj.mu_x^2-obj.mu_y^2)-obj.mu_y*(obj.mu_z-obj.lambda_0)+1/2*obj.beta_1c*obj.mu_x*obj.mu_y)) ...
                        +obj.theta_t*(-1/5*obj.alpha_1c*(1-obj.e^5) ...
                            +1/4*(1-obj.e^4)*(obj.beta_0p*obj.mu_y+obj.beta_0*obj.mu_x) ...
                            +1/6*(1-obj.e^3)*(1/2*obj.beta_1s*(obj.mu_x^2-obj.mu_y^2) ...
                                +obj.beta_1c*obj.mu_x*obj.mu_y ...
                                -2*obj.mu_y*(obj.mu_z-obj.lambda_0))) ... %fixed
                        +1/2*(1-obj.e^4)*obj.alpha_1c*obj.beta_0p ...
                        +(1-obj.e^2)*(obj.beta_0*obj.mu_x*(obj.mu_z-obj.lambda_0) ...
                            -1/2*obj.beta_0*obj.beta_1c*(obj.mu_x^2+1/2*obj.mu^2) ... %fixed
                            +1/2*obj.beta_1s*obj.beta_0*obj.mu_x*obj.mu_y) ...
                        -2/3*(1-obj.e^3)*(obj.alpha_1c*(obj.mu_z-obj.lambda_0) ...
                            +obj.beta_0*obj.beta_0p*obj.mu_x ...
                            -obj.delta*obj.mu_y/obj.a_0 ...
                            +1/4*obj.beta_1s*(obj.alpha_1c*obj.mu_y-obj.alpha_1s*obj.mu_x) ...
                            +1/4*obj.beta_1c*(obj.alpha_1s*obj.mu_y-3*obj.alpha_1c*obj.mu_x));

            C_QA1s  = obj.theta_1c*(1/8*(1-obj.e^2)*obj.beta_0*(obj.mu_x^2-obj.mu_y^2) ...
                            +1/12*(1-obj.e^3)*(obj.mu_x*(obj.beta_1s-obj.alpha_1c)-(obj.alpha_1s+obj.beta_1c)*obj.mu_y)) ...
                        +obj.theta_1s*(1/4*obj.beta_0p*(1-obj.e^4) ...
                            +(1-obj.e^3)*(-1/3*(obj.mu_z-obj.lambda_0) ...
                                -1/4*(obj.alpha_1s*obj.mu_x+obj.beta_1s*obj.mu_y) ...
                                +1/12*(obj.beta_1c*obj.mu_x-obj.alpha_1c*obj.mu_y))...
                            -1/4*(1-obj.e^2)*obj.beta_0*obj.mu_x*obj.mu_y) ...
                        +obj.theta_0*(-1/4*(1-obj.e^4)*obj.alpha_1s ...
                            +1/3*(1-obj.e^3)*(obj.beta_0p*obj.mu_x-obj.beta_0*obj.mu_y) ...
                            +1/2*(1-obj.e^2)*(1/4*obj.beta_1c*(obj.mu_x^2-obj.mu_y^2)-obj.mu_x*(obj.mu_z-obj.lambda_0)-1/2*obj.beta_1s*obj.mu_x*obj.mu_y)) ...
                        +obj.theta_t*(-1/5*obj.alpha_1s*(1-obj.e^5) ...
                            +1/4*(1-obj.e^4)*(obj.beta_0p*obj.mu_x-obj.beta_0*obj.mu_y) ...
                            +1/6*(1-obj.e^3)*(1/2*obj.beta_1c*(obj.mu_x^2-obj.mu_y^2) ...
                                -obj.beta_1s*obj.mu_x*obj.mu_y ...
                                -2*obj.mu_x*(obj.mu_z-obj.lambda_0))) ... %fixed
                        +1/2*(1-obj.e^4)*obj.alpha_1s*obj.beta_0p ...
                        +(1-obj.e^2)*(-obj.beta_0*obj.mu_y*(obj.mu_z-obj.lambda_0) ...
                            -1/2*obj.beta_0*obj.beta_1s*(obj.mu_y^2+1/2*obj.mu^2) ...
                            +1/2*obj.beta_1c*obj.beta_0*obj.mu_x*obj.mu_y) ...
                        -2/3*(1-obj.e^3)*(obj.alpha_1s*(obj.mu_z-obj.lambda_0) ...
                            -obj.beta_0*obj.beta_0p*obj.mu_y ...
                            -obj.delta*obj.mu_x/obj.a_0 ...
                            +1/4*obj.beta_1s*(-obj.alpha_1c*obj.mu_x+3*obj.alpha_1s*obj.mu_y) ...
                            +1/4*obj.beta_1c*(-obj.alpha_1s*obj.mu_x+obj.alpha_1c*obj.mu_y));
            % 计算惯性力矩系数及其一阶谐波分量
            C_QI0   = -obj.I_bar_beta*(obj.beta_0*obj.beta_0p-obj.Omega_dot/obj.Omega^2+1/2*obj.epsilon_1c*(obj.beta_1c-obj.p_bar_H)+1/2*obj.epsilon_1s*(obj.beta_1s+obj.q_bar_H));
            C_QI1c  = obj.eta_y*obj.M_bar_beta - obj.I_bar_beta*(obj.beta_0*obj.epsilon_1c+obj.beta_0p*(obj.beta_1c-obj.p_bar_H));
            C_QI1s  = obj.eta_x*obj.M_bar_beta - obj.I_bar_beta*(obj.beta_0*obj.epsilon_1s+obj.beta_0p*(obj.beta_1s+obj.q_bar_H));
            
            % 计算合力矩C_Q及其一阶谐波分量
            C_Q0    = obj.s*obj.a_0/(2*obj.b)*C_QA0  + C_QI0;
            C_Q1c   = obj.s*obj.a_0/(2*obj.b)*C_QA1c + C_QI1c;
            C_Q1s   = obj.s*obj.a_0/(2*obj.b)*C_QA1s + C_QI1s;
            
            % 计算合力矩Q及其一阶谐波分量
            Q_0     = obj.rho*(obj.Omega*obj.R)^2*obj.A*obj.R*C_Q0;
            Q_1c    = obj.rho*(obj.Omega*obj.R)^2*obj.A*obj.R*C_Q1c;
            Q_1s    = obj.rho*(obj.Omega*obj.R)^2*obj.A*obj.R*C_Q1s;
            
            % 计算在桨毂系下的力矩
            L_h     = -obj.b/2*(obj.K_beta*obj.beta_1s+Q_0*obj.beta_1c+Q_1c*obj.beta_0);
            M_h     = obj.b/2*(obj.beta_1s*Q_0+obj.beta_0*Q_1s-obj.K_beta*obj.beta_1c);
            N_h     = obj.b*Q_0;
            
            L_h   = L_h*obj.direction; %镜像转换
            N_h   = N_h*obj.direction; %镜像转换
            
            % 计算在体轴系下的力矩
            obj.L   = L_h*cos(obj.gamma_s)-N_h*sin(obj.gamma_s)+obj.h_R*obj.Y;
            obj.M   = M_h-obj.h_R*obj.X+obj.x_cg*obj.Z;
            obj.N   = L_h*sin(obj.gamma_s)+N_h*cos(obj.gamma_s)-obj.x_cg*obj.Y;
            
%            disp(['L:', num2str(obj.L), ' M:', num2str(obj.M),' N:', num2str(obj.N)])
        
        end
    end
end

