classdef LinkData < handle
    properties
        Joint
        DH_Parent % DH Parameter from parent's frame to the joint
        DH_Child  % DH Parameter from the joint to child's frame 
        Local_Frame
        Dynamics
        Gravity
        Secondary_Frames
        Points
    end
    
    methods
        
        function obj = LinkData(ID,q,qd,qdd,rho,DH_Parent,DH_Child,m,cm,I)
            if nargin==1
                obj.Joint.q = 0;
                obj.Joint.qd = 0;
                obj.Joint.qdd = 0;
                obj.Joint.rho = 1;
                obj.Joint.force = {};
                obj.Joint.moment = {};
                obj.Joint.input = {};
                
                obj.DH_Parent = zeros(1,4);
                obj.DH_Child = zeros(1,4);
                
                obj.Local_Frame = Frame;
                obj.Dynamics.m = 0;
                obj.Dynamics.cm = zeros(3,1);
                obj.Dynamics.I = zeros(3);
                
                obj.Secondary_Frames = {};
                obj.Points = {};
                obj.Gravity = ID;
                return
            end
            obj.Joint.q = q;
            obj.Joint.qd = qd;
            obj.Joint.qdd = qdd;
            obj.Joint.rho = rho;
            obj.Joint.force = {};
            obj.Joint.moment = {};
            obj.Joint.input = {};
            
            obj.DH_Parent = DH_Parent;
            obj.DH_Child = DH_Child;
            
            obj.Local_Frame = Frame({},ID);
            
            obj.Dynamics.m = m;
            obj.Dynamics.cm = cm;
            obj.Dynamics.I = I;
            obj.Gravity = {};
            
            obj.Secondary_Frames = {};
            obj.Points = {};
            
        end
        
        function [q,qd,qdd,rho] = getJointInfo(obj)
            q = obj.Joint.q;
            qd = obj.Joint.qd;
            qdd = obj.Joint.qdd;
            rho = obj.Joint.rho;
        end
        
        function [p,v,a,R,w,dw] = getFrameInfo(obj)
            linear = obj.Local_Frame.Linear;
            angular = obj.Local_Frame.Angular;
            p = linear.p;
            v = linear.v;
            a = linear.a;
            R = angular.R;
            w = angular.w;
            dw = angular.dw;
            
        end
        
        function [m,cm,I] = getDynamicParameter(obj)
            m = obj.Dynamics.m;
            cm = obj.Dynamics.cm;
            I = obj.Dynamics.I;
        end
        
        function updateKinematics(obj,p,v,a,R,w,dw)
            linear.p = p;
            linear.v = v;
            linear.a = a;
            angular.R = R;
            angular.w = w;
            angular.dw = dw;
            obj.Local_Frame.updateKinematics(linear,angular);
        end
        
        function updateDynamics(obj,f,n,tau) % joint dynamics/ not dynamics parameter
            obj.Joint.force = f;
            obj.Joint.moment = n;
            obj.Joint.input = tau;
        end
        
        function addFrame(obj,frame,rel_pose)
            if isequal(class(frame),'char')
                frame =  Frame(obj.Local_Frame,frame,rel_pose);
            else
                frame.Parent_Frame = obj.Local_Frame;
            end
            obj.Secondary_Frames{end+1} = frame;
        end
        
        function addPoint(obj,point,rel_p)
            if isequal(class(point),'char')
                point =  Point(obj.Local_Frame,point,rel_p);
            else
                point.Parent_Frame = obj.Local_Frame;
            end
            obj.Points{end+1} = point;
        end
    end
    
end