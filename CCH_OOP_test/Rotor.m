classdef Rotor < handle
    %Rotor 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties (GetAccess = public)      
        R;
        a_0;
        delta;
        u_R,v_R,w_R;
        V;
        Omega;
        A;
        
    end
    
    properties 
        FX;
    end
    
    properties (Dependent)
        dp1;
        dp2;
    end
    
    methods
        function obj = Rotor()
            %Rotor 构造此类的实例
            obj.R=1;
        end
        
        function cal_FX(obj)
            obj.FX=obj.a_0*obj.R;
        end
        
        
        function get_R(obj)
            disp(['R: ', num2str(obj.R)])
        end
        
        function obj = set_R(obj,R)
            obj.R = R;
        end
        
        function dp1 = get.dp1(obj)
            dp1=obj.dp2+10;
        end
        function dp2 = get.dp2(obj)
            dp2=pi*obj.R^2;
        end
        
        
    end
end

