classdef FuselageZhang < Helicopter
    %FUSELAGE 机身
    %   
    properties (GetAccess = protected)
        s_F;    % 机身有效面积 19.3?
        x_F;
        y_F;
        z_F;
        R;      % 旋翼半径
        
        Omega;
    end
    properties (GetAccess = public)
        isEnable;
    end
    properties (Dependent)
        A;      % 旋翼面积
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
        function obj = FuselageZhang()
            %FUSELAGE 构造此类的实例
        end
        function A  = get.A(obj)
            A = pi*obj.R^2;
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
            xs_alpha_CLF=[-10.0986,-7.93095,-6.0119,-4.01,-2.00857,-0.00619048,2.0781,4.08,5.99857,8.08381,10.0024,12.0048,14.0905];
            CLF=[-0.0291429,-0.0274286,-0.0231429,-0.0197143,-0.0171429,-0.0128571,-0.0111429,-0.00771429,-0.00428571,-8.57E-04,0.00257143,0.00685714,0.0111429];
            C_LF=interp1(xs_alpha_CLF,CLF,rad2deg(obj.alpha_F),'spline');

            xs_CLF=[0.012	0.00942857	0.00514286	0.00171429	-0.00342857	-0.00685714	-0.0102857	-0.0145714	-0.0188571	-0.0231429	-0.0274286];
            CDF=[0.0242429	0.0233952	0.0217048	0.0216857	0.0224905	0.0233048	0.0249524	0.0265952	0.0282381	0.029881	0.0323571];
            C_DF=interp1(xs_CLF,CDF,C_LF,'spline');

            xs_beta_CYF=[-14.9607	-9.95511	-4.85971	-2.48597	0	2.50281	5	10.0954	15.0842];
            CYF=[0.0376543	0.0216049	0.00876543	0.00487654	0	-0.00802469	-0.015	-0.0278395	-0.0407407];
            C_YF=interp1(xs_beta_CYF,CYF,rad2deg(obj.beta_F),'spline');
            
            X_F = -1/2*obj.rho*obj.V_inf^2*obj.A*C_DF;
            Y_F = 1/2*obj.rho*obj.V_inf^2*obj.A*C_YF;
            Z_F = -1/2*obj.rho*obj.V_inf^2*obj.A*C_LF;
            
            obj.X       = X_F;
            obj.Y       = Y_F;
            obj.Z       = Z_F;
        end
        
        function calculate_torque(obj)
            % calculate_torque
            
            xs_alpha_CMF=[-10.3333	-8.25	-6.25	-4.16667	-2.25	-0.25	1.83333	3.83333	5.75	7.83333	9.75	11.75	13.75];
            CMF=[-0.0062069	-0.00551724	-0.00448276	-0.0037931	-0.00275862	-0.00103448	-6.90E-04	6.90E-04	0.00172414	0.00275862	0.00344828	0.00448276	0.00517241];
            C_MF=interp1(xs_alpha_CMF,CMF,rad2deg(obj.alpha_F),'spline');

            xs_beta_CRF=[-14.2432	-9.25539	-4.26762	-1.72483	0.72016	3.37853	5.71683	10.9135	15.9013];
            CRF=[-8.44E-04	-4.79E-04	-1.15E-04	1.10E-04	2.49E-04	3.94E-04	5.31E-04	9.41E-04	0.00130562];
            C_RF=interp1(xs_beta_CRF,CRF,rad2deg(obj.beta_F),'spline');

            xs_beta_CNF=[-14.9164	-9.92281	-4.83368	-2.34242	0	2.64564	5.03582	10.2482	15.1519];
            CNF=[0.0382407	0.0244349	0.0127673	0.00690287	0.00E+00	-0.00586439	-0.0128283	-0.0275504	-0.0445327];
            C_NF=interp1(xs_beta_CNF,CNF,rad2deg(obj.beta_F),'spline');

            L_F = 1/2*obj.rho*obj.V_inf^2*obj.A*2*obj.R*C_RF;
            M_F = 1/2*obj.rho*obj.V_inf^2*obj.A*2*obj.R*C_MF;
            N_F = 1/2*obj.rho*obj.V_inf^2*obj.A*2*obj.R*C_NF;
            
            
            obj.L = L_F;
            obj.M = M_F;
            obj.N = N_F;
        end
    end
end

