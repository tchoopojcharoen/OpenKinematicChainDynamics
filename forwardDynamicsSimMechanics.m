function qdd = forwardDynamicsSimMechanics(q,qd,u,g,robot)
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
sys = 'testModel';
new_system(sys);
load_system(sys);


%I/O
add_block('built-in/Constant',[sys '/u']);
set_param([sys '/u'],'Value',['[' num2str(u',pre) ']''']);
add_block('simulink/Commonly Used Blocks/Out1',[sys '/qdd']);
add_block('built-in/Demux',[sys '/uDemux']);
set_param('testModel/uDemux','Outputs',num2str(N));
add_block('built-in/Mux',[sys '/qddMux']);
set_param('testModel/qddMux','Inputs',num2str(N));

add_line(sys,'qddMux/1','qdd/1');
add_line(sys,'u/1','uDemux/1');

% create a base

add_block('built-in/Subsystem',[sys '/Base']);
ref = load_system('LinkBase');
Simulink.BlockDiagram.copyContentsToSubSystem(ref,[ sys '/Base' ]);
set_param([sys '/Base/Mechanism Configuration'],'GravityVector',['[' num2str(g',pre) ']']);
set_param([sys '/Base/Child'],'Side','right');

% create and connect each link
for i = 1:N,
    linkName = sprintf('Link %d',i);
    add_block('built-in/Subsystem',[sys '/' linkName])
    if rho(i)
        ref = 'LinkFDR.slx';
    else
        ref = 'LinkFDP.slx';
    end
    ref = load_system(ref);
    Simulink.BlockDiagram.copyContentsToSubSystem(ref,[ sys '/' linkName]);
    child = [linkName '/LConn1'];
    if ~parent_idx(i)
        parent = 'Base/RConn1';
    else
        parent = [sprintf('Link %d',parent_idx(i)) '/RConn1'];
    end
    add_line(sys,parent,child);
    add_block('built-in/terminator',[sys '/terminator q' num2str(i) ]);
    add_block('built-in/terminator',[sys '/terminator qd' num2str(i) ]);
    
    add_line(sys,sprintf('uDemux/%d',i),[linkName '/1']);
    add_line(sys,[linkName '/1'],['terminator q' num2str(i) '/1']);
    add_line(sys,[linkName '/2'],['terminator qd' num2str(i) '/1']);
    add_line(sys,[linkName '/3'],sprintf('qddMux/%d',i));
    if rho(i)
        joint_name = 'Revolute Joint';
    else
        joint_name = 'Prismatic Joint';
    end
    set_param([sys '/Link ' num2str(i) '/' joint_name] ,'q',num2str(q(i)',pre))
    set_param([sys '/Link ' num2str(i) '/' joint_name] ,'qd',num2str(qd(i)',pre))
    set_param([sys '/Link ' num2str(i) '/' joint_name '/Joint'],'PositionTargetPriority','High')
    set_param([sys '/Link ' num2str(i) '/' joint_name '/Joint'],'VelocityTargetPriority','High')
    
    maskobj = Simulink.Mask.create([sys '/' linkName]);
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

set_param(sys,'StopTime','1');
set_param(sys,'SimMechanicsOpenEditorOnUpdate','off');
set_param(sys,'SolverPrmCheckMsg','none');
[~,~,qdd_t] = sim(sys);
close_system(sys,0);
qdd = qdd_t(1,:)';


end