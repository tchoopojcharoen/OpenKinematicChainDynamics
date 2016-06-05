clear all;
clc;
testLink;

u = inverseDynamicsSimMechanics(zeros(4,1),zeros(4,1),zeros(4,1),g,robot);
qdd = forwardDynamicsSimMechanics(zeros(4,1),zeros(4,1),u,g,robot);
simulinkModel = robotModelSimMechanics('Test_Robot',zeros(4,1),zeros(4,1),g,robot);