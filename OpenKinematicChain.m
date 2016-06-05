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
                close_system(simulinkModel);
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
            
            set_param(simulinkModel,'SimMechanicsOpenEditorOnUpdate','off');
            set_param(simulinkModel,'SolverPrmCheckMsg','none');
            save_system(simulinkModel,name);
            close_system(simulinkModel);
            
            obj.Model = simulinkModel;
            
        end
        
        function simulinkModel = generateInverseDynamicsController(obj,K_0,K_1)
            precision = 15;
            pre = sprintf('%%.%df ',precision);
            N = obj.DOF;
            rho = obj.Parameters.Kinematics.rho;
            parent_idx = obj.Parameters.Kinematics.parent_idx;
            DH_p = obj.Parameters.Kinematics.DH_p;
            DH_c = obj.Parameters.Kinematics.DH_c;
            m = obj.Parameters.Dynamics.m;
            cm = obj.Parameters.Dynamics.cm;
            I = obj.Parameters.Dynamics.I;
            g = obj.Gravity;
            % create a new Simulink Model
            simulinkModel = [obj.Name '_Inverse_Dynamics_Controller'];
            if exist(simulinkModel)==4,
                close_system(simulinkModel,0);
            end
            
            new_system(simulinkModel);
            load_system(simulinkModel);
            %I/O
            
            add_block('simulink/Commonly Used Blocks/In1',[simulinkModel '/q']);
            add_block('simulink/Commonly Used Blocks/In1',[simulinkModel '/qd']);
            add_block('simulink/Commonly Used Blocks/In1',[simulinkModel '/q_r']);
            add_block('simulink/Commonly Used Blocks/In1',[simulinkModel '/qd_r']);
            add_block('simulink/Commonly Used Blocks/In1',[simulinkModel '/qdd_r']);
            add_block('simulink/Commonly Used Blocks/Out1',[simulinkModel '/u']);
            
            %% Inverse Dynamics Subsystem
            invDyn = [simulinkModel '/Inverse Dynamics'];
            add_block('built-in/Subsystem',invDyn);
            
            add_block('simulink/Commonly Used Blocks/In1',[invDyn '/q']);
            add_block('simulink/Commonly Used Blocks/In1',[invDyn '/qd']);
            add_block('simulink/Commonly Used Blocks/In1',[invDyn '/qdd']);
            add_block('simulink/Commonly Used Blocks/Out1',[invDyn '/u']);
            add_block('built-in/Demux',[invDyn '/qDemux']);
            set_param([invDyn '/qDemux'],'Outputs',num2str(N));
            add_block('built-in/Demux',[invDyn '/qdDemux']);
            set_param([invDyn '/qdDemux'],'Outputs',num2str(N));
            add_block('built-in/Demux',[invDyn '/qddDemux']);
            set_param([invDyn '/qddDemux'],'Outputs',num2str(N));
            add_block('built-in/Mux',[invDyn '/uMux']);
            set_param([invDyn '/uMux'],'Inputs',num2str(N));
            
            add_line(invDyn,'q/1','qDemux/1');
            add_line(invDyn,'qd/1','qdDemux/1');
            add_line(invDyn,'qdd/1','qddDemux/1');
            add_line(invDyn,'uMux/1','u/1');
            
            % create a base
            
            add_block('built-in/Subsystem',[invDyn '/Base']);
            ref = load_system('LinkBase');
            Simulink.BlockDiagram.copyContentsToSubSystem(ref,[ invDyn '/Base' ]);
            set_param([invDyn '/Base/Mechanism Configuration'],'GravityVector',['[' num2str(g',pre) ']']);
            set_param([invDyn '/Base/Child'],'Side','right');
            
            % create and connect each link
            for i = 1:N,
                linkName = sprintf('Link %d',i);
                add_block('built-in/Subsystem',[invDyn '/' linkName])
                if rho(i)
                    ref = 'LinkIDR.slx';
                else
                    ref = 'LinkIDP.slx';
                end
                ref = load_system(ref);
                Simulink.BlockDiagram.copyContentsToSubSystem(ref,[ invDyn '/' linkName]);
                child = [linkName '/LConn1'];
                if ~parent_idx(i)
                    parent = 'Base/RConn1';
                else
                    parent = [sprintf('Link %d',parent_idx(i)) '/RConn1'];
                end
                add_line(invDyn,parent,child);
                add_line(invDyn,sprintf('qDemux/%d',i),[linkName '/1']);
                add_line(invDyn,sprintf('qdDemux/%d',i),[linkName '/2']);
                add_line(invDyn,sprintf('qddDemux/%d',i),[linkName '/3']);
                add_line(invDyn,[linkName '/1'],sprintf('uMux/%d',i));
                
                maskobj = Simulink.Mask.create([invDyn '/' linkName]);
                Prompt = {'DH Parent','DH Child','Mass','Center of Mass','Inertia'};
                Name = {'DH_p','DH_c','m','cm','I'};
                DH_p_str = ['[' num2str(DH_p(i,:),pre) ']'];
                DH_c_str = ['[' num2str(DH_c(i,:),pre) ']'];
                m_str = num2str(m(i),pre);
                cm_str = ['[' num2str(cm(:,i)',pre) ']'''];
                temp = num2str(I(:,:,i),pre);
                I_str = ['[' temp(1,:) ';' temp(2,:) ';' temp(3,:) ']'];
                
                Value = {DH_p_str,DH_c_str,m_str,cm_str,I_str};
                for j = 1:numel(Prompt),
                    maskobj.addParameter('Type','edit','Prompt',Prompt{j},'Name',Name{j},'Value',Value{j});
                end
            end
            
            %% Outer loop Controller Subsystem
            outerloop = [ simulinkModel '/Outer Loop Controller'];
            add_block('built-in/Subsystem',outerloop);
            Simulink.BlockDiagram.copyContentsToSubSystem('OuterLoopController',outerloop);
            maskobj = Simulink.Mask.create(outerloop);
            Prompt = {'Proportional Gain Matrix','Derivative Gain Matrix'};
            Names = {'K_0','K_1'};
            K_0_str = num2str(K_0(:,:),pre);
            K_1_str = num2str(K_1(:,:),pre);
            K_str = {K_0_str,K_1_str};
            
            for j = 1:numel(Prompt),
                Value = '[';
                temp = K_str{j};
                for k = 1:N-1
                    Value = [Value temp(k,:) ';'];
                end
                Value = [Value temp(N,:) ']'];
                maskobj.addParameter('Type','edit','Prompt',Prompt{j},'Name',Names{j},'Value',Value);
            end
            %% add lines
            add_line(simulinkModel,'q/1','Outer Loop Controller/1');
            add_line(simulinkModel,'qd/1','Outer Loop Controller/2');
            add_line(simulinkModel,'q_r/1','Outer Loop Controller/3');
            add_line(simulinkModel,'qd_r/1','Outer Loop Controller/4');
            add_line(simulinkModel,'qdd_r/1','Outer Loop Controller/5');
            
            add_line(simulinkModel,'q/1','Inverse Dynamics/1');
            add_line(simulinkModel,'qd/1','Inverse Dynamics/2');
            add_line(simulinkModel,'Outer Loop Controller/1','Inverse Dynamics/3');
            add_line(simulinkModel,'Inverse Dynamics/1','u/1');
            
            %% simulation configuration
            set_param(simulinkModel,'SimMechanicsOpenEditorOnUpdate','off');
            set_param(simulinkModel,'SolverPrmCheckMsg','none');
            
            save_system(simulinkModel);
            close_system(simulinkModel);
            
        end
        
        function [simulinkModel,t,q,qd,qdd,u] = simulate(obj,q_0,qd_0,control,refTraj,t_max)
            precision = 15;
            pre = sprintf('%%.%df ',precision);
            
            N = obj.DOF;
            rho = obj.Parameters.Kinematics.rho;
            %% Create New Simulink Model
            
            simulinkModel = [obj.Name '_Closed_Loop_Controller'];
            if exist(simulinkModel)==4,
                close_system(simulinkModel,0);
            end
            
            new_system(simulinkModel);
            load_system(simulinkModel);
            
            
            %% Reference Trajectory
            add_block('built-in/Subsystem',[simulinkModel '/' refTraj]);
            load_system(refTraj) % TO DO: check whether it is in directory
            Simulink.BlockDiagram.copyContentsToSubSystem(refTraj,[simulinkModel '/' refTraj]);
            
            %% Controller
            add_block('built-in/Subsystem',[simulinkModel '/' control]);
            load_system(control)
            Simulink.BlockDiagram.copyContentsToSubSystem(control,[simulinkModel '/' control]);
            
            %% Dynamics
            if isempty(obj.Model)
                obj.Model = obj.generateSimMechanicsModel(q_0,qd_0);
                load_system(obj.Model)
            else
                load_system(obj.Model)
                for i = 1:N,
                    if rho(i)
                        joint_name = 'Revolute Joint';
                    else
                        joint_name = 'Prismatic Joint';
                    end
                    set_param([obj.Model '/' obj.Model '/Link ' num2str(i) '/' joint_name] ,'q',num2str(q_0(i)',pre))
                    set_param([obj.Model '/' obj.Model '/Link ' num2str(i) '/' joint_name] ,'qd',num2str(qd_0(i)',pre))
                    set_param([obj.Model '/' obj.Model '/Link ' num2str(i) '/' joint_name '/Joint'],'PositionTargetPriority','Low')
                    set_param([obj.Model '/' obj.Model '/Link ' num2str(i) '/' joint_name '/Joint'],'VelocityTargetPriority','Low')
                    
                end
            end
            add_block('built-in/Subsystem',[simulinkModel '/' obj.Model]);
            Simulink.BlockDiagram.copyContentsToSubSystem(obj.Model,[simulinkModel '/' obj.Model]);
            
            %% Outputs
            
            add_block('simulink/Commonly Used Blocks/Out1',[simulinkModel '/q']);
            add_block('simulink/Commonly Used Blocks/Out1',[simulinkModel '/qd']);
            add_block('simulink/Commonly Used Blocks/Out1',[simulinkModel '/qdd']);
            add_block('simulink/Commonly Used Blocks/Out1',[simulinkModel '/u']);
            
            %% Add connections
            add_line(simulinkModel,[obj.Model '/1'],'q/1');
            add_line(simulinkModel,[obj.Model '/2'],'qd/1');
            add_line(simulinkModel,[obj.Model '/3'],'qdd/1');
            add_line(simulinkModel,[obj.Model '/1'],[control '/1']);
            add_line(simulinkModel,[obj.Model '/2'],[control '/2']);
            add_line(simulinkModel,[refTraj '/1'],[control '/3']);
            add_line(simulinkModel,[refTraj '/2'],[control '/4']);
            add_line(simulinkModel,[refTraj '/3'],[control '/5']);
            add_line(simulinkModel,[control '/1'],'u/1');
            add_line(simulinkModel,[control '/1'],[obj.Model '/1']);
            
            %% Simulation
            set_param(simulinkModel,'StopTime',num2str(t_max,pre));
            set_param(simulinkModel,'SolverPrmCheckMsg','none');
            save_system(simulinkModel);
            
            pause(2);
            set_param(simulinkModel,'SimMechanicsOpenEditorOnUpdate','off');
            close_system(simulinkModel,1);
            load_system(simulinkModel);
            [t,~,q,qd,qdd,u] = sim(simulinkModel);
            obj.inverseDynamics(q(end,:)',qd(end,:)',qdd(end,:)');
        end
    end
    
end