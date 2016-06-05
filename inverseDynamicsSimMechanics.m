function u = inverseDynamicsSimMechanics(q,qd,qdd,g,robot)
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
add_block('built-in/Constant',[sys '/q']);
set_param([sys '/q'],'Value',['[' num2str(q',pre) ']''']);
add_block('built-in/Constant',[sys '/qd']);
set_param([sys '/qd'],'Value',['[' num2str(qd',pre) ']''']);
add_block('built-in/Constant',[sys '/qdd']);
set_param([sys '/qdd'],'Value',['[' num2str(qdd',pre) ']''']);
add_block('simulink/Commonly Used Blocks/Out1',[sys '/u']);

add_block('built-in/Demux',[sys '/qDemux']);
set_param('testModel/qDemux','Outputs',num2str(N));
add_block('built-in/Demux',[sys '/qdDemux']);
set_param('testModel/qdDemux','Outputs',num2str(N));
add_block('built-in/Demux',[sys '/qddDemux']);
set_param('testModel/qddDemux','Outputs',num2str(N));
add_block('built-in/Mux',[sys '/uMux']);
set_param('testModel/uMux','Inputs',num2str(N));

add_line(sys,'q/1','qDemux/1');
add_line(sys,'qd/1','qdDemux/1');
add_line(sys,'qdd/1','qddDemux/1');
add_line(sys,'uMux/1','u/1');

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
        ref = 'LinkIDR.slx';
    else
        ref = 'LinkIDP.slx';
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
    add_line(sys,sprintf('qDemux/%d',i),[linkName '/1']);
    add_line(sys,sprintf('qdDemux/%d',i),[linkName '/2']);
    add_line(sys,sprintf('qddDemux/%d',i),[linkName '/3']);
    add_line(sys,[linkName '/1'],sprintf('uMux/%d',i));
    
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

set_param(sys,'StopTime','0.001');
set_param(sys,'SimMechanicsOpenEditorOnUpdate','off');
set_param(sys,'SolverPrmCheckMsg','none');
[~,~,u_t] = sim(sys);
close_system(sys,0);
u = u_t(end,:)';
end