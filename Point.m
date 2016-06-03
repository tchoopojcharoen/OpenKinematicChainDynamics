classdef Point < handle
    properties
        ID
        Parent_Frame = {};
        Relative_Position = {};
        Linear = {};        
    end
    
    methods
        
        function obj = Point(parent,ID,rel_p)
            obj.ID = ID;
            obj.Relative_Position = rel_p;
            % parent frame, not link
            
            obj.Parent_Frame = parent;
        end
        
        function updateKinematics(obj)
            frame = obj.Parent_Frame;
            
            if isempty(frame.Linear)
                frame.updateKinematics
            end
            p = frame.Linear.p;
            v = frame.Linear.v;
            a = frame.Linear.a;
            
            R = frame.Angular.R;
            w = frame.Angular.w;
            dw = frame.Angular.dw;
            
            s = obj.Relative_Position;
            linear.p = p + R*s;
            linear.v = v + skew(R*s)'*w;
            linear.a = a + skew(R*s)'*dw + skew(skew(R*s)'*w)'*w;
            obj.Linear = linear;
        end
    end    
end