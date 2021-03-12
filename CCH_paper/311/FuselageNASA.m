classdef Fuselage < Helicopter
    %FUSELAGE 机身
    %   
    properties (GetAccess = protected)
        s_F;    % 机身有效面积 19.3?
        x_F;
        y_F;
        z_F;
        Omega;  % 旋翼转速
        rho;	% 大气密度
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
        V;
        mu;
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

        VT; 	% blade tip speed
        
        Power_total;
    end
    
    methods
        function obj = Fuselage()
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
        function V = get.V(obj)
        	V = sqrt(obj.u_F^2+obj.v_F^2+obj.w_F^2)
        end
        function mu = get.mu(obj)
        	mu = obj.V*cos(obj.alpha_F)/obj.VT;
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
        function VT = get.VT(obj)
        	VT = obj.Omega*obj.R;
        end
        function Power_total = get.Power_total(obj)
            Power_total = obj.rho*obj.s_F*(sqrt(obj.U^2+obj.V^2+obj.W^2))^3;
        end
        
        function calculate_force(obj)
            % calculate_force
            % C_LF
            
            array_alpha_F = deg2rad([-6 -1 4]);
            array_mu = [0.1 0.21 0.31];
            matrix_C_LF = [	0.0095 		0.101 		0.0105;
            				0.008 		0.0102 		0.0122;
            				8.29e-4 	5.16e-3 	9.3e-3]; % every line is a same mu
            C_LF = interp2(array_alpha_F, array_mu, matrix_C_LF,deg2rad(obj.alpha_F),obj.mu,'spline');

           	array_beta_F = deg2rad([-5 0 5]);
           	matrix_C_YF = [	0.0004 		0.0001 		-0.0001;
           					0.0007 		0.0002 		-0.0005;
           					0.00125 	0.00025 	-0.0008];
           	C_YF = interp2(array_beta_F, array_mu, matrix_C_YF,deg2rad(obj.beta_F),obj.mu,'spline');
			
			Y 		= C_YF*obj.rho*obj.A*obj.VT^2;
			Z 		= C_LF*obj.rho*obj.A*obj.VT^2;

			obj.Y 	= Y;
			obj.Z 	= Z;            
        end
        
        function calculate_torque(obj)
            % calculate_torque

            array_beta_F = deg2rad([-5 0 5]);
            array_mu = [0.1 0.21 0.31];
            matrix_C_lF = [	0.00018 	-0.00001 	-0.0002;
            				0.00023 	0.000005 	-0.00023;
            				0.0003 		0 			-0.00028;];
            matrix_C_nF = [	-0.0001		0			0.00004;
            				-0.0001		0.00001		0.00017;
            				-0.00012	0.00001		0.00017;];

            C_lF = interp2(array_beta_F, array_mu, matrix_C_lF,deg2rad(obj.beta_F),obj.mu,'spline');
            C_nF = interp2(array_beta_F, array_mu, matrix_C_nF,deg2rad(obj.beta_F),obj.mu,'spline');

            array_alpha_F = deg2rad([-6 -1 4]);
            matrix_C_mF = [	-0.0002		0.0001		0.0003;
            				-0.0004		-0.00003	0.0004;
            				9.46e-4		3.36e-4		2.6e-5;];
            C_mF = interp2(array_alpha_F, array_mu, matrix_C_mF,deg2rad(obj.beta_F),obj.mu,'spline');

            L_F = C_lF*obj.rho*obj.A*obj.VT^2*obj.R;
            M_F = C_mF*obj.rho*obj.A*obj.VT^2*obj.R;
            N_F = C_nF*obj.rho*obj.A*obj.VT^2*obj.R;

            obj.L = L_F;
            obj.M = M_F;
            obj.N = N_F; % in body axe
        end
    end
end

