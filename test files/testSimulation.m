dynamics = @(t,x)[x(5:8);forwardDynamics(x(1:4),x(5:8),zeros(4,1))];
tic;
[t,x] = ode45(dynamics,0:0.01:10,[zeros(4,1);zeros(4,1)]);
toc % 130 seconds
q = x(:,1:4)';
qd = x(:,5:8)';
qdd = zeros(4,size(x,1));
for i = 1:size(x,1)
    qdd(:,i) = forwardDynamics(q(:,i),qd(:,i),zeros(4,1));
end
