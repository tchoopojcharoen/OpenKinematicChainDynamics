function qdd = forwardDynamics(q,qd,u)
N = size(q,1);
g = [0;0;-9.80655];
b = inverseDynamics(q,qd,zeros(4,1),g);
H = zeros(N);
for i = 1:N,
    e = zeros(N,1);
    e(i) = 1;
    h = inverseDynamics(q,zeros(N,1),e,zeros(3,1));
    H(:,i) = h;
end

qdd = H\(u-b);

end