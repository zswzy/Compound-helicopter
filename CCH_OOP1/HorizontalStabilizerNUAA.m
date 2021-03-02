classdef HorizontalStabilizerNUAA < Helicopter
    %HORIZONTALTAILNUAA
    %   此处显示详细说明
    
    properties (GetAccess = protected)
        s_HS;
        s_e;
        x_HS;
        y_HS;
        z_HS;
        
    end
    
    properties (GetAccess = public)
        delta_e;
        
        inteference;
        
        C_LFitted;
        C_DFitted;
        C_MFitted;
    end
    
    properties (Dependent)
        q_HS;
        u_HS;
        v_HS;
        w_HS;
        
        alpha_HS; % 迎角
        beta_HS;  % 侧滑角
    end
    
    methods
        %--------------构造函数---------------
        function obj = HorizontalStabilizerNUAA()
            %HORIZONTALTAIL 构造此类的实例 
            % C_LFitted
            list_alpha_HS = deg2rad([-10	-6	-4	-1	0]);
            list_alpha_F = deg2rad([-0.6 -4.6]);
            matrix_C_L = [0.0102	0.01022	0.01021	0.01015	0.0102;
                            0.0105	0.01052	0.01053	0.0105	0.0105];
            [obj.C_LFitted, ~] = SurfaceFitting(list_alpha_HS, list_alpha_F, matrix_C_L,'thinplateinterp');
            
            % C_DFitted
            list_alpha_HS = deg2rad([-10 -6 -4 -1 0 5 10]);
            list_alpha_F = deg2rad([-0.6 -4.6]);
            matrix_C_D = 1e-3 * [0.1563	0.1172	0.0625	0.0156	0	0.0156	0.1094;
                            1.2031     1.2344	1.2266	1.2422	1.2344	1.1094	0.6016];
            [obj.C_DFitted, ~] = SurfaceFitting(list_alpha_HS, list_alpha_F, matrix_C_D,'thinplateinterp');
            
            % C_MFitted
            list_alpha_HS = deg2rad([-10	-6	-4	-1	0	5	10]);
            list_alpha_F = deg2rad([-0.6 -4.6]);
            matrix_C_M = 1e-4 * [2	2.1042	2.0208	1.9792	1.9375	1	0.1042;
                                    4	3.7872	3.4894	3	2.4681	1.3617	0.9681];
            [obj.C_MFitted, ~] = SurfaceFitting(list_alpha_HS, list_alpha_F, matrix_C_M,'thinplateinterp');
        end
        %--------------Dependent变量---------------
        function q_HS = get.q_HS(obj)
            q_HS = 1/2*obj.rho*(obj.u_HS^2+obj.v_HS^2+obj.w_HS^2);
        end
        function u_HS = get.u_HS(obj)
            u_HS = obj.u + obj.q*obj.z_HS - obj.r*obj.y_HS;
        end
        function v_HS = get.v_HS(obj)
            v_HS = obj.v + obj.r*obj.x_HS - obj.p*obj.z_HS;
        end
        function w_HS = get.w_HS(obj)
            w_HS = obj.w + obj.p*obj.y_HS - obj.q*obj.x_HS;
        end
        function alpha_HS    = get.alpha_HS(obj)
            if obj.u_HS == 0
                alpha_HS = pi/2*sign(obj.w_HS);
            elseif obj.u_HS > 0
                alpha_HS = atan(obj.w_HS/obj.u_HS);
            else
                alpha_HS = pi + atan(obj.w_HS/obj.u_HS);
            end
        end
        function beta_HS    = get.beta_HS(obj)
            if obj.u_HS == 0
                beta_HS = pi/2*sign(obj.v_HS);
            elseif obj.u_HS > 0
                beta_HS = atan(obj.v_HS/obj.u_HS);
            else
                beta_HS = pi + atan(obj.v_HS/obj.u_HS);
            end
        end
        %--------------方法---------------
        function [fitresult, gof] = SurfaceFitting(x,y,z,fittype)

            %SurfaceFitting 二维数据拟合
            %   z是一个表
            %   每个x对应z的每一列
            %   每个y对应z的每一行
            %   fittype：拟合类型
            [xData, yData, zData] = prepareSurfaceData(x,y,z);
            % Set up fittype and options.


            ft = fittype;

            % Fit model to data.
            [fitresult, gof] = fit( [xData, yData], zData, ft, 'Normalize', 'on' );
            
%             % Plot fit with data.
%             figure( 'Name', ['fitted ' inputname(3)] );
%             h = plot( fitresult, [xData, yData], zData );
% 
%             legend( h, 'fitted data', [inputname(3) ' vs. ' inputname(1) ' ,' inputname(2)], 'Location', 'NorthEast', 'Interpreter', 'none' );
% 
%             % Label axes
%             xlabel( inputname(1), 'Interpreter', 'none' );
%             ylabel( inputname(2), 'Interpreter', 'none' );
%             zlabel( ['fitted ' inputname(3)], 'Interpreter', 'none' );
%             grid on
%             view( -52.1, 33.3 );

            end

        
        function calculate_force(obj, alpha_F)
            %calculate_force 
            
            % 计算气动力系数
            C_X     = -obj.C_DFitted(obj.alpha_HS, alpha_F);
            C_Z     = -obj.C_LFitted(obj.alpha_HS, alpha_F);
            
            % 计算平尾坐标系下的气动力
            X_HS    = obj.q_HS*obj.s_HS*C_X;
            Y_HS    = 0;
            Z_HS    = obj.q_HS*obj.s_HS*C_Z;
            
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
        function calculate_torque(obj, alpha_F)
            % 计算气动力矩系数
            C_M     = obj.C_MFitted(obj.alpha_HS, alpha_F);
            
            % 计算平尾坐标系下的气动力矩
            L_HS    = 0;
            M_HS    = obj.q_HS*obj.s_HS*C_M;
            N_HS    = 0;
            
            % 计算机体坐标系下的气动力矩
            obj.L   = cos(obj.alpha_HS)*cos(obj.beta_HS) * L_HS ...
                        -cos(obj.alpha_HS)*sin(obj.beta_HS) * M_HS ...
                        -sin(obj.alpha_HS) * N_HS ...
                        +obj.y_HS*obj.Z-obj.z_HS*obj.Y;
            obj.M   = sin(obj.beta_HS) * L_HS ...
                        +cos(obj.beta_HS) * M_HS ...
                        +obj.z_HS*obj.X-obj.x_HS*obj.Z;
            obj.N   = sin(obj.alpha_HS)*cos(obj.beta_HS) * L_HS ...
                        -sin(obj.alpha_HS)*sin(obj.beta_HS) * M_HS ...
                        +cos(obj.alpha_HS) * N_HS ...
                        +obj.x_HS*obj.Y-obj.y_HS*obj.X;
        end
    end
end

