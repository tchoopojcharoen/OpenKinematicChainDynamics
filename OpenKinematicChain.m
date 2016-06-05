classdef OpenKinematicChain < handle
    properties
        Name;
        DOF;
        Parameters;
        Links;
        Model = [];
        Gravity;
    end
    
    methods
        
        function obj = OpenKinematicChain(name,parent_idx,rho,DH_p,DH_c,m,cm,I,g)
            obj.Name = name;
            N = numel(rho);
            data = cell(1,N);
            IDs = cell(1,N);
            % q, qd, qdd initially are zeros
            q = zeros(N,1);
            qd = zeros(N,1);
            qdd = zeros(N,1);
            
            for k = 1:N,
                IDs{k} = ['link ' num2str(k) ];
                data{k} = LinkData(IDs{k},q(k),qd(k),qdd(k),rho(k),DH_p(k,:),DH_c(k,:),m(k),cm(:,k),I(:,:,k));
            end
            base = Link(g);
            linkList = cell(1,k+1);
            linkList{1} = base;
            for k = 1:N,
                if ~parent_idx(k)
                    link = Link(base,IDs{k},data{k});
                    linkList{k+1} = link;
                else
                    link = Link(linkList{parent_idx(k)+1},IDs{k},data{k});
                    linkList{k+1} = link;
                end
                obj.Links = linkList;
                obj.Gravity = g;
                obj.DOF = N;
                kinematics.rho = rho;
                kinematics.DH_p = DH_p;
                kinematics.DH_c = DH_c;
                kinematics.parent_idx = parent_idx;
                
                dynamics.m = m;
                dynamics.cm = cm;
                dynamics.I = I;
                
                obj.Parameters.Kinematics = kinematics;
                obj.Parameters.Dynamics = dynamics;
            end
        end
        
        function clearKinematics(obj)
            N = obj.DOF;
            for k = 1:N,
                
                obj.Links{k+1}.Data.Local_Frame.Linear = {};
                obj.Links{k+1}.Data.Local_Frame.Angular = {};
                % TO DO: clear all attached points and frames
            end
        end
        
        function clearKinematicsAndDynamics(obj)
            N = obj.DOF;
            for k = 1:N,
                joint.force = {};
                joint.moment = {};
                joint.input = {};
                obj.Links{k+1}.Data.Joint.force = {};
                obj.Links{k+1}.Data.Joint.moment = {};
                obj.Links{k+1}.Data.Joint.input = {};
                obj.Links{k+1}.Data.Local_Frame.Linear = {};
                obj.Links{k+1}.Data.Local_Frame.Angular = {};
                % TO DO: clear all attached points and frames
            end
        end
        
        function forwardKinematics(obj,q,qd,qdd)
            N = obj.DOF;
            obj.clearKinematics;
            for k = 1:N,
                obj.Links{k+1}.Data.Joint.q = q(k);
                obj.Links{k+1}.Data.Joint.qd = qd(k);
                obj.Links{k+1}.Data.Joint.qdd = qdd(k);
            end
            obj.Links{1}.forwardKinematics;
        end
        
        function u = inverseDynamics(obj,q,qd,qdd)
            N = obj.DOF;
            obj.clearKinematicsAndDynamics;
            for k = 1:N,
                obj.Links{k+1}.Data.Joint.q = q(k);
                obj.Links{k+1}.Data.Joint.qd = qd(k);
                obj.Links{k+1}.Data.Joint.qdd = qdd(k);
            end
            obj.Links{1}.inverseDynamics;
            
            u = zeros(N,1);
            for k = 1:N,
                u(k) = obj.Links{k+1}.Data.Joint.input;
            end
        end
        
        function qdd = forwardDynamics(obj,q,qd,u)
            N = obj.DOF;
            obj.clearKinematicsAndDynamics;
            for k = 1:N,
                obj.Links{k+1}.Data.Joint.q = q(k);
                obj.Links{k+1}.Data.Joint.qd = qd(k);
                obj.Links{k+1}.Data.Joint.input = u(k);
            end
            
            parent_idx = obj.Parameters.Kinematics.parent_idx;
            rho = obj.Parameters.Kinematics.rho;
            DH_p = obj.Parameters.Kinematics.DH_p;
            DH_c = obj.Parameters.Kinematics.DH_c;
            
            m = obj.Parameters.Dynamics.m;
            cm = obj.Parameters.Dynamics.cm;
            I = obj.Parameters.Dynamics.I;
            
            robotNoG = OpenKinematicChain('dummy',parent_idx,rho,DH_p,DH_c,m,cm,I,zeros(3,1));
            b = obj.inverseDynamics(q,qd,zeros(4,1));
            H = zeros(N);
            for i = 1:N,
                e = zeros(N,1);
                e(i) = 1;
                h = robotNoG.inverseDynamics(q,zeros(N,1),e);
                H(:,i) = h;
            end
            
            qdd = H\(u-b);
            
            obj.forwardKinematics(q,qd,qdd);
            
            for k = 1:N,
                obj.Links{k+1}.Data.Joint.qdd = qdd(k);
                obj.Links{k+1}.Data.Joint.input = u(k);
            end
            uu = obj.inverseDynamics(q,qd,qdd);
        end
        
        function simulinkModel = generateSimMechanicsModel(obj,q_0,qd_0)
            precision = 15;
            pre = sprintf('%%.%df ',precision);
            
            N = obj.DOF;
            name = obj.Name;
            g = obj.Gravity;
            rho = obj.Parameters.Kinematics.rho;
            parent_idx = obj.Parameters.Kinematics.parent_idx;
            DH_p = obj.Parameters.Kinematics.DH_p;
            DH_c = obj.Parameters.Kinematics.DH_c;
            m = obj.Parameters.Dynamics.m;
            cm = obj.Parameters.Dynamics.cm;
            I = obj.Parameters.Dynamics.I;
            % create a new Simulink Model
            simulinkModel = name;
            if exist(simulinkModel)==4,
                delete_system(simulinkModel);
            end
            
            new_system(simulinkModel);
            load_system(simulinkModel);
            
            %I/O
            add_block('simulink/Commonly Used Blocks/In1',[simulinkModel '/u']);
            add_block('simulink/Commonly Used Blocks/Out1',[simulinkModel '/q']);
            add_block('simulink/Commonly Used Blocks/Out1',[simulinkModel '/qd']);
            add_block('simulink/Commonly Used Blocks/Out1',[simulinkModel '/qdd']);
            
            robotSubsystem = [simulinkModel '/' name];
            add_block('built-in/Subsystem',robotSubsystem);
            
            add_block('simulink/Commonly Used Blocks/In1',[robotSubsystem '/u']);
            add_block('simulink/Commonly Used Blocks/Out1',[robotSubsystem '/q']);
            add_block('simulink/Commonly Used Blocks/Out1',[robotSubsystem '/qd']);
            add_block('simulink/Commonly Used Blocks/Out1',[robotSubsystem '/qdd']);
            add_line(simulinkModel,'u/1',[name '/1']);
            add_line(simulinkModel,[name '/1'],'q/1');
            add_line(simulinkModel,[name '/2'],'qd/1');
            add_line(simulinkModel,[name '/3'],'qdd/1');
            
            add_block('built-in/Demux',[robotSubsystem '/uDemux']);
            set_param([robotSubsystem '/uDemux'],'Outputs',num2str(N));
            
            add_block('built-in/Mux',[robotSubsystem '/qMux']);
            set_param([robotSubsystem '/qMux'],'Inputs',num2str(N));
            add_block('built-in/Mux',[robotSubsystem '/qdMux']);
            set_param([robotSubsystem '/qdMux'],'Inputs',num2str(N));
            add_block('built-in/Mux',[robotSubsystem '/qddMux']);
            set_param([robotSubsystem '/qddMux'],'Inputs',num2str(N));
            
            add_line(robotSubsystem,'qMux/1','q/1');
            add_line(robotSubsystem,'qdMux/1','qd/1');
            add_line(robotSubsystem,'qddMux/1','qdd/1');
            add_line(robotSubsystem,'u/1','uDemux/1');
            
            % create a base
            
            add_block('built-in/Subsystem',[robotSubsystem '/Base']);
            ref = load_system('LinkBase');
            Simulink.BlockDiagram.copyContentsToSubSystem(ref,[ robotSubsystem '/Base' ]);
            set_param([robotSubsystem '/Base/Mechanism Configuration'],'GravityVector',['[' num2str(g',pre) ']']);
            set_param([robotSubsystem '/Base/Child'],'Side','right');
            
            % create and connect each link
            for i = 1:N,
                linkName = sprintf('Link %d',i);
                add_block('built-in/Subsystem',[robotSubsystem '/' linkName])
                if rho(i)
                    ref = 'LinkFDR.slx';
                else
                    ref = 'LinkFDP.slx';
                end
                ref = load_system(ref);
                Simulink.BlockDiagram.copyContentsToSubSystem(ref,[ robotSubsystem '/' linkName]);
                child = [linkName '/LConn1'];
                if ~parent_idx(i)
                    parent = 'Base/RConn1';
                else
                    parent = [sprintf('Link %d',parent_idx(i)) '/RConn1'];
                end
                add_line(robotSubsystem,parent,child);
                add_block('built-in/terminator',[robotSubsystem '/terminator q' num2str(i) ]);
                add_block('built-in/terminator',[robotSubsystem '/terminator qd' num2str(i) ]);
                
                add_line(robotSubsystem,sprintf('uDemux/%d',i),[linkName '/1']);
                add_line(robotSubsystem,[linkName '/1'],sprintf('qMux/%d',i));
                add_line(robotSubsystem,[linkName '/2'],sprintf('qdMux/%d',i));
                add_line(robotSubsystem,[linkName '/3'],sprintf('qddMux/%d',i));
                if rho(i)
                    joint_name = 'Revolute Joint';
                else
                    joint_name = 'Prismatic Joint';
                end
                set_param([robotSubsystem '/Link ' num2str(i) '/' joint_name] ,'q',num2str(q_0(i)',pre))
                set_param([robotSubsystem '/Link ' num2str(i) '/' joint_name] ,'qd',num2str(qd_0(i)',pre))
                set_param([robotSubsystem '/Link ' num2str(i) '/' joint_name '/Joint'],'PositionTargetPriority','Low')
                set_param([robotSubsystem '/Link ' num2str(i) '/' joint_name '/Joint'],'VelocityTargetPriority','Low')
                
                maskobj = Simulink.Mask.create([robotSubsystem '/' linkName]);
                Prompt = {'DH Parent','DH Child','Mass','Center of Mass','Inertia'};
                Names = {'DH_p','DH_c','m','cm','I'};
                DH_p_str = ['[' num2str(DH_p(i,:),pre) ']'];
                DH_c_str = ['[' num2str(DH_c(i,:),pre) ']'];
                m_str = num2str(m(i),pre);
                cm_str = ['[' num2str(cm(:,i)',pre) ']'''];
                temp = num2str(I(:,:,i),pre);
                I_str = ['[' temp(1,:) ';' temp(2,:) ';' temp(3,:) ']'];
                
                Value = {DH_p_str,DH_c_str,m_str,cm_str,I_str};
                for j = 1:numel(Prompt),
                    maskobj.addParameter('Type','edit','Prompt',Prompt{j},'Name',Names{j},'Value',Value{j});
                end
            end
            
            set_param(simulinkModel,'StopTime','10');
            set_param(simulinkModel,'SimMechanicsOpenEditorOnUpdate','off');
            set_param(simulinkModel,'SolverPrmCheckMsg','none');
            save_system(simulinkModel,name);
            close_system(simulinkModel);
            
            obj.Model = simulinkModel;
            
        end
    end
    
end