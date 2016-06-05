rho = [1;1;1];
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

I1 = 0.1*eye(3);
I2 = 0.2*eye(3);
I3 = 0.3*eye(3);

DH_c = [        pi/2    d1      a1      pi/2    ;
                pi/2    0       0       pi/2    ;
                0       d3      a3      0       ];
m = [m1;m2;m3];
cm      = [    -xc1    -xc2    -xc3        ;
                -yc1    0       0          ;
                0       zc2     0          ]; 
I = cat(3,cat(3,I1,I2),I3);
g = [0;0;-9.80655];

q = zeros(N,1);
qd = zeros(N,1);
qdd = zeros(N,1);
q_0 = zeros(N,1);
qd_0 = zeros(N,1);

test_robot = SerialManipulator('Test_Robot',rho,DH_c,m,cm,I,g);
%% Forward Kinemaitcs
test_robot.forwardKinematics(q,qd,qdd);
%% Inverse Dynamics
u = test_robot.inverseDynamics(q,qd,qdd);
%% Forward Dynamics
qdd = test_robot.forwardDynamics(q,qd,u);
%% SimMechanics Model Generation
modelName = test_robot.generateSimMechanicsModel(q_0,qd_0);