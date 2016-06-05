%% move the cursor to a section and press ctrl+enter to run that section alone

rho = [1;1;1;1];
N = numel(rho);
d1 = 1.5;
a1 = 0.85;
d3 = 1;
a3 = 1.2;
d4 = 1.1;
a4 = 0.9;
d5 = 0.5;
xc1 = 0.2*a1;
xc2 = 0.5*d4;
xc3 = 0.5*a3;
xc4 = 0.5*a4;
yc1 = 0.3*d1;
zc2 = 0.25*d3;

m1 = 1;
m2 = 2;
m3 = 3;
m4 = 4;

I1 = 0.1*eye(3);
I2 = 0.2*eye(3);
I3 = 0.3*eye(3);
I4 = 0.4*eye(3);

DH_p = [        0       0       0       0       ;
                0       0       0       0       ;
                0       0       0       0       ;
                pi/2    d5      0       pi/2    ];
DH_c = [        pi/2    d1      a1      pi/2    ;
                pi/2    0       0       pi/2    ;
                0       d3      a3      0       ;
                pi/2    -d4     a4      0       ];
m = [m1;m2;m3;m4];
cm      = [    -xc1    -xc2    -xc3    -xc4    ;
                -yc1    0       0       0       ;
                0       zc2     0       0       ]; 
I = cat(3,cat(3,cat(3,I1,I2),I3),I4);
g = [0;0;-9.80655];
% base(0) <- link1
% link1   <- link2
% link2   <- link3
% link2   <- link4

parent_idx = [0 1 2 2];
q = zeros(N,1);
qd = zeros(N,1);
qdd = zeros(N,1);
q_0 = zeros(N,1);
qd_0 = zeros(N,1);

test_robot = OpenKinematicChain('Test_Robot',parent_idx,rho,DH_p,DH_c,m,cm,I,g);
%% Forward Kinemaitcs
test_robot.forwardKinematics(q,qd,qdd);
%% Inverse Dynamics
u = test_robot.inverseDynamics(q,qd,qdd);
%% Forward Dynamics
qdd = test_robot.forwardDynamics(q,qd,u);
%% SimMechanics Model Generation
modelName = test_robot.generateSimMechanicsModel(q_0,qd_0);
%% Controller Generation and Closed Loop Control Simulation
K_0 = 100*eye(N);
K_1 = 50*eye(N);
t_max = 10;
controllerName = test_robot.generateInverseDynamicsController(K_0,K_1);
[controlledSystem,t,q,qd,qdd,u] = test_robot.simulate(q_0,qd_0,controllerName,'ReferenceTrajectoryExample',t_max);