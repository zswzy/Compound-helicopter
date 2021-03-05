classdef Helicopter < handle
    %HELICOPTER 此处显示有关此类的摘要
    properties (GetAccess = public)
        rho;    % 大气密度
        
        GW;
        GWF;
        
        Ix;
        Iy;
        Iz;
    end
    properties (GetAccess = public)
        % 姿态
        theta;
        phi;
        psi;
        
        % 线速度/线加速度在牵连垂直系下的分量
        U;
        V;
        W;
        U_dot;
        V_dot;
        W_dot;
        
        % 线速度/线加速度在体轴系下的分量
        u;
        v;
        w;
        u_dot;
        v_dot;
        w_dot;
        
        % 角速度/角加速度在体轴系下的分量
        p;
        q;
        r;
        p_dot;
        q_dot;
        r_dot;
        
        % 气动力在体轴系下的分量
        X; 
        Y;
        Z;
        % 气动力距在体轴系下的分量
        L;
        M;
        N;
    end
    
    methods
        function obj    = Helicopter()
            %HELICOPTER 构造此类的实例

        end
        function calculate_local_variables(obj)
            obj.u = cos(obj.theta)*cos(obj.psi)*obj.U ...
               +cos(obj.theta)*sin(obj.psi)*obj.V ...
               -sin(obj.theta)*obj.W;
            obj.v = (sin(obj.theta)*sin(obj.phi)*cos(obj.psi)-cos(obj.phi)*sin(obj.psi))*obj.U ...
               +(sin(obj.theta)*sin(obj.phi)*sin(obj.psi)+cos(obj.phi)*cos(obj.psi))*obj.V ...
               +sin(obj.phi)*cos(obj.theta)*obj.W;
            obj.w = (sin(obj.theta)*cos(obj.phi)*cos(obj.psi)+sin(obj.phi)*sin(obj.psi)-sin(obj.phi)*cos(obj.psi))*obj.U ...
               +sin(obj.theta)*cos(obj.phi)*sin(obj.psi)*obj.V ...
               +cos(obj.phi)*cos(obj.theta)*obj.W; 
           
            obj.u_dot = cos(obj.theta)*cos(obj.psi)*obj.U_dot ...
               +cos(obj.theta)*sin(obj.psi)*obj.V_dot ...
               -sin(obj.theta)*obj.W_dot;
            obj.v_dot = (sin(obj.theta)*sin(obj.phi)*cos(obj.psi)-cos(obj.phi)*sin(obj.psi))*obj.U_dot ...
               +(sin(obj.theta)*sin(obj.phi)*sin(obj.psi)+cos(obj.phi)*cos(obj.psi))*obj.V_dot ...
               +sin(obj.phi)*cos(obj.theta)*obj.W_dot;
            obj.w_dot = (sin(obj.theta)*cos(obj.phi)*cos(obj.psi)+sin(obj.phi)*sin(obj.psi)-sin(obj.phi)*cos(obj.psi))*obj.U_dot ...
               +sin(obj.theta)*cos(obj.phi)*sin(obj.psi)*obj.V_dot ...
               +cos(obj.phi)*cos(obj.theta)*obj.W_dot; 
        end
        function calculate_force(obj)
           obj.X = - obj.GWF*sin(obj.theta);
           obj.Y = obj.GWF*cos(obj.theta)*sin(obj.phi);
           obj.Z = obj.GWF*cos(obj.theta)*cos(obj.phi);
        end
        function calculate_torque(obj)
            obj.L = 0;
            obj.M = 0;
            obj.N = 0;
        end
        
    end
end

