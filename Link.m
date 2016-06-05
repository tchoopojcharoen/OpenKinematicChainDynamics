classdef Link < handle
    properties
        ID;
        Data;
        
        Parent = {};
        Child = {};
        Brother = {};
        
    end
    
    methods
        
        function obj = Link(parent,ID,data)
            
            if nargin==1
                obj.ID = 'base';
                obj.Data = LinkData(parent);
                return
            end
            obj.ID = ID;
            obj.Data = data;
            %get parent node
            if isequal(class(parent),'char')
                parent = obj.getChildByID(parent);
            end
            
            obj.Parent = parent;
            obj.Data.Gravity = parent.Data.Gravity;
            if isempty(parent.Child)
                parent.Child = obj;
            else
                parent.Child.addBrother(obj);
            end
            
        end
        
        function addBrother(obj,brother)
            if isempty(obj.Brother)
                obj.Brother = brother;
            else
                obj.Brother.addBrother(brother);
            end
        end
        
        %Find the link with a given id, return [] if not found
        function link = getChildByID(obj,ID)
            link = [];
            
            if isequal(ID,obj.ID)
                link = obj;
                return;
            end
            
            
            link = obj.Child.getChildByID(ID);
            if isempty(link)
                link = obj.Child.Brother.getChildByID(ID);
            end
            if ~isempty(link)
                return
            end
        end
        
        function forwardKinematics(obj)
                     
            if isempty(obj.Parent)
                obj.Child.forwardKinematics;
                return
            end
            
            if isempty(obj.Parent.Data.Local_Frame)
                obj.Parent.forwardKinematics;
            end
            
            z = [0;0;1];
            
            [q,qd,qdd,rho] = obj.Data.getJointInfo;
            DH_p = obj.Data.DH_Parent;
            DH_c = obj.Data.DH_Child;
            
            [p,v,a,R,w,dw] = obj.Parent.Data.getFrameInfo;
            
            %
            h = R*rot(DH_p(1),'z')*[DH_p(3) 0 DH_p(2)]';    % parent to joint
            if rho
                R_temp = rot(q,'z');
                d_temp = q;
            else
                R_temp = eye(3);
                d_temp = 0;
            end
            R = R*rot(DH_p(1),'z')*rot(DH_p(4),'x');
            r = R*R_temp*rot(DH_c(1),'z')*[DH_c(3) 0 d_temp+DH_c(2)]';    % joint to child
            
            p_j = p + h;
            v_j = v + skew(h)'*w;
            a_j = a + skew(h)'*dw + skew(skew(h)'*w)'*w;
            
            p = p_j + r;
            w = w + rho*R*z*qd;
            v = v_j + skew(r)'*w + (1-rho)*R*z*qd;
            dw = dw + rho*(R*z*qdd + skew(R*z*qd)'*w);
            a = a_j + skew(r)'*dw + skew(skew(r)'*w)'*w + (1-rho)*(R*z*qdd+2*skew(R*z*qd)'*w);
            R = R*R_temp*rot(DH_c(1),'z')*rot(DH_c(4),'x');
            
            %linear components
            
            obj.Data.updateKinematics(p,v,a,R,w,dw);
            if ~isempty(obj.Child)
                obj.Child.forwardKinematics;
            end
            if ~isempty(obj.Brother)
               obj.Brother.forwardKinematics;
            end
        end
        
        function inverseDynamics(obj)
            
            if isempty(obj.Parent) % if the link is the base
                if ~isempty(obj.Child)
                    obj.Child.inverseDynamics;
                end
                return
            end
            
            if isempty(obj.Data.Local_Frame.Linear)
                obj.forwardKinematics;
            end
            
            [~,~,a,R,w,dw] = obj.Data.getFrameInfo;
            [q,~,~,rho] = obj.Data.getJointInfo;
            [~,~,~,R_p,~,~] = obj.Parent.Data.getFrameInfo;
            DH_p = obj.Data.DH_Parent;
            DH_c = obj.Data.DH_Child;
            R_p_m = R_p*rot(DH_p(1),'z')*rot(DH_p(4),'x');
            if rho
                R_temp = rot(q,'z');
                d_temp = q;
            else
                R_temp = eye(3);
                d_temp = 0;
            end
            
            r = R_p_m*R_temp*rot(DH_c(1),'z')*[DH_c(3) 0 d_temp+DH_c(2)]';    % joint to child
            [m,cm,I] = obj.Data.getDynamicParameter;
            a_cm = a + skew(R*cm)'*dw + skew(skew(R*cm)'*w)'*w;
            
            f_sum = zeros(3,1);
            n_sum = zeros(3,1);
            
            if ~isempty(obj.Child)
                if isempty(obj.Child.Data.Joint.force)
                    obj.Child.inverseDynamics
                end
                
                DH_p_c = obj.Child.Data.DH_Parent;
                h = R*rot(DH_p_c(1),'z')*[DH_p_c(3) 0 DH_p_c(2)]';
                f_c = obj.Child.Data.Joint.force;
                n_c = obj.Child.Data.Joint.moment;
                f_sum = f_sum + f_c;
                n_sum = n_sum + n_c + skew(R*cm-h)'*f_c;
                
                temp = obj.Child;
                while ~isempty(temp.Brother)
                    temp = temp.Brother;
                    if isempty(temp.Data.Joint.force)
                        temp.inverseDynamics
                    end
                    DH_p_b = temp.Data.DH_Parent;
                    h = R*rot(DH_p_b(1),'z')*[DH_p_b(3) 0 DH_p_b(2)]';
                    f_b = temp.Data.Joint.force;
                    n_b = temp.Data.Joint.moment;
                    f_sum = f_sum + f_b;
                    n_sum = n_sum + n_b + skew(R*cm-h)'*f_b;
                end
            end
            g = obj.Data.Gravity;
            f = f_sum + m*(a_cm - g);
            n = n_sum + skew(r+R*cm)*f + R*I*R'*w + skew(R*I*R'*w)'*w;
            tau = (rho*n'+(1-rho)*f')*R_p_m;
            obj.Data.updateDynamics(f,n,tau(3));
            
            if ~isempty(obj.Parent)
                if isempty(obj.Parent.Data.Joint.force)&&~isempty(obj.Parent.Parent)
                    obj.Parent.inverseDynamics;
                end
            end
            
        end
        
    end
    
end