classdef PropellerNUAA < Helicopter
    %PROPELLERNUAA 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties (GetAccess = protected)
        delta;
        e_oswald;
        s;
        x_PR;
        y_PR;
        z_PR;
        
        Omega;
        R;
    end
    
    properties (GetAccess = public)
        isEnable;   % 1:on（default） 0:off
        
        theta_0
    end
    
    properties (Dependent)
        mu;
        mu_x;
        u_PR; 
        v_PR;
        w_PR;
        lambda_0;
        v_0;
        
        A;
        C_X;
        T;
        Power_total;
        Power_induced;
        Power_resist; 
    end
    
    methods
        function obj = PropellerNUAA()
            %PROPELLERNUAA 构造此类的实例
        end
        function mu = get.mu(obj)
            mu = sqrt(obj.U^2+obj.V^2+obj.W^2)/(obj.Omega*obj.R);
        end
        function mu_x = get.mu_x(obj)
            mu_x = obj.u_PR/(obj.Omega*obj.R);
        end
        function u_PR = get.u_PR(obj)
            u_PR = obj.u + obj.q*obj.z_PR - obj.r*obj.y_PR;
        end
        function v_PR = get.v_PR(obj)
            v_PR = obj.v + obj.r*obj.x_PR - obj.p*obj.z_PR;
        end
        function w_PR = get.w_PR(obj)
            w_PR = obj.w + obj.p*obj.y_PR - obj.q*obj.x_PR;
        end
        function lambda_0 = get.lambda_0(obj)
            lambda_0 = fsolve(@(x) x-abs(obj.C_X)/(2*sqrt(obj.mu^2+(obj.mu_x+x)^2)),1,optimset('Display','off'));
        end
        function v_0 = get.v_0(obj)
            v_0 = obj.lambda_0*(obj.Omega*obj.R);
        end
        function A              = get.A(obj)
           A = pi*obj.R^2; 
        end
        function C_X = get.C_X(obj)
            C_X = obj.X/(obj.rho*(obj.Omega*obj.R)^2*obj.A);
        end
        function T = get.T(obj)
            T = sqrt(obj.X^2+obj.Y^2+obj.Z^2);
        end
        function Power_total    = get.Power_total(obj)
            Power_total = obj.Power_induced + obj.Power_resist;
        end
        function Power_induced  = get.Power_induced(obj)
            Power_induced = obj.T*obj.v_0 / obj.e_oswald;
        end
        function Power_resist   = get.Power_resist(obj)
            Power_resist = obj.rho*obj.A*obj.s*(obj.Omega*obj.R)^3*obj.delta/8*(1+3*obj.mu^2);
        end
        function calculate_force(obj)
            %calculate_force 计算XYZ力
            theta_0PR_list = deg2rad([4	8	12	16	20	24	28	32	36])';
            X_PR_list = [291	873	1673	2327	3055	4000	5455	8146	10909]';
            %X_PRFitted = fit(theta_0PR_list, X_PR_list, 'linearinterp');
            
            %X_H = X_PRFitted(obj.theta_0);
            X_H = interp1(theta_0PR_list,X_PR_list,obj.theta_0,'spline');
            Y_H = 0;
            Z_H = 0;
            
            obj.X = X_H;
            obj.Y = Y_H;
            obj.Z = Z_H;

        end
        function calculate_torque(obj)
            theta_0PR_list = deg2rad([4	8	12	16	20	24	28	32	36])';
            L_PR_list = [318.18	545.45	909.09	1272.7	1909.1	2546.5	3363.6	4477.3	6954.5]';
            %L_PRFitted = fit(theta_0PR_list, L_PR_list, 'linearinterp');

            %obj.L = L_PRFitted(obj.theta_0) + (obj.y_PR*obj.Z-obj.z_PR*obj.Y);
            obj.L = interp1(theta_0PR_list,L_PR_list,obj.theta_0,'spline') ...
                    + (obj.y_PR*obj.Z-obj.z_PR*obj.Y);
            obj.M = obj.z_PR*obj.X-obj.x_PR*obj.Z;
            obj.N = obj.x_PR*obj.Y-obj.y_PR*obj.X;
        end
    end
end

