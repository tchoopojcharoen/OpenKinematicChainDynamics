function simulinkModel = robotModelSimMechanics(name,q_0,qd_0,g,robot)
precision = 15;
pre = sprintf('%%.%df ',precision);
N = size(q,1);
rho = robot.Parameters.Kinematics.rho;
parent_idx = robot.Parameters.Kinematics.parent_idx;
DH_p = robot.Parameters.Kinematics.DH_p;
DH_c = robot.Parameters.Kinematics.DH_c;
m = robot.Parameters.Dynamics.m;
cm = robot.Parameters.Dynamics.cm;
I = robot.Parameters.Dynamics.I;
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

set_param(simulinkModel,'StopTime','10');
set_param(simulinkModel,'SimMechanicsOpenEditorOnUpdate','off');
set_param(simulinkModel,'SolverPrmCheckMsg','none');
save_system(simulinkModel,name);
close_system(simulinkModel);


end