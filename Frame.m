classdef Frame < handle
    properties
        ID
        Parent_Frame = {};
        Relative_Pose = {};
        Linear = {};
        Angular = {};
        
    end
    
    methods
        
        function obj = Frame(parent,ID,rel_pose)
            if nargin==0
                obj.ID = 'base';
                
                linear.p = zeros(3,1);
                linear.v = zeros(3,1);
                linear.a = zeros(3,1);
                
                angular.R = eye(3);
                angular.w = zeros(3,1);
                angular.dw = zeros(3,1);
                
                obj.Linear = linear;
                obj.Angular = angular;
                
                obj.Relative_Pose.p = zeros(3,1);
                obj.Relative_Pose.R = eye(3);
                return
            end
            
            if isempty(parent)
                obj.ID = ID;
                obj.Relative_Pose.p = zeros(3,1);
                obj.Relative_Pose.R = eye(3);
                return
            end
            obj.ID = ID;
            obj.Relative_Pose = rel_pose;
            % parent frame, not link
            
            obj.Parent_Frame = parent;
        end
        
        function updateKinematics(obj,varargin)
            frame = obj.Parent_Frame;
            if isempty(frame) % the current frame is local
                if isempty(obj.Linear) % kinematics hasn't been updated
                    if isempty(varargin)
                        error('local frame has not been updated')
                    else
                        frame.Linear = varargin{1};
                        frame.Angular = varargin{2};
                    end
                else
                    frame.Linear = obj.Linear;
                    frame.Angular = obj.Angular;
                end
            else
                if isempty(frame.Linear) % kinematics hasn't been updated                   
                    frame.updateKinematics
                end
                
            end
            p = frame.Linear.p;
            v = frame.Linear.v;
            a = frame.Linear.a;
            
            R = frame.Angular.R;
            w = frame.Angular.w;
            dw = frame.Angular.dw;
            
            s = obj.Relative_Pose.p;
            
            linear.p = p + R*s;
            linear.v = v + skew(R*s)'*w;
            linear.a = a + skew(R*s)'*dw + skew(skew(R*s)'*w)'*w;
            obj.Linear = linear;
            obj.Angular = frame.Angular;
            obj.Angular.R = R*obj.Relative_Pose.R;
        end
        
    end
end