function zOut = zTrajectory (t)
% Z parameters
z0 = 0.548901719911339;
Tau = 0.08;
A = 0.67 - 0.548901719911339;
for i=1:size(t,2)
z(i) = A*(1-exp(-t(i)/Tau)) + z0;
zdot(i) = A*exp(-t(i)/Tau)*(t(i)/Tau);
zddot(i) = -A*exp(-t(i)/Tau)*(t(i)/Tau)^2;
end

zOut.t =t;
zOut.pos = z;
zOut.vel = zdot;
zOut.acc = zddot; 
end
