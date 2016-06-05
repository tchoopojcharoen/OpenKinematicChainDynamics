q = [0;0;0;0];
qd = [0;0;0;0];
qdd = [0;0;0;0];
rho = [1;1;1;1];
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
% base  <- link1
% link1 <- link2
% link2 <- link3
% link2 <- link4


N = 4;
data = cell(1,N);
IDs = {'link1','link2','link3','link4'};
parent_idx = [0 1 2 2];
tic;
for k = 1:N,
    data{k} = LinkData(IDs{k},q(k),qd(k),qdd(k),rho(k),DH_p(k,:),DH_c(k,:),m(k),cm(:,k),I(:,:,k));
end

base = Link(g);
link1 = Link(base,'link1',data{1});
link2 = Link(link1,'link2',data{2});
link3 = Link(link2,'link3',data{3});
link4 = Link(link2,'link4',data{4});
base.forwardKinematics;
base.inverseDynamics;
toc

robot.parent_idx = parent_idx;
robot.rho = rho;
robot.DH_p = DH_p;
robot.DH_c = DH_c;
robot.m = m;
robot.cm = cm;
robot.I = I;
