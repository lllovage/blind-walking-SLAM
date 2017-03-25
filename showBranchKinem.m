function showBranchKinem(boundKinemMap)
% figure(3)
% plot(boundKinemMap.t(1:end), boundKinemMap.l1(1,:),'r')
% hold on;
% plot(boundKinemMap.t(1:end), boundKinemMap.l1(2,:),'b')
% plot(boundKinemMap.t(1:end), boundKinemMap.l1(3,:),'g')
% plot(boundKinemMap.t(1:end), boundKinemMap.l3(1,:),'r')
% plot(boundKinemMap.t(1:end), boundKinemMap.l3(2,:),'b')
% plot(boundKinemMap.t(1:end), boundKinemMap.l3(3,:),'g')
% plot(boundKinemMap.t(1:end), boundKinemMap.l5(1,:),'r')
% plot(boundKinemMap.t(1:end), boundKinemMap.l5(2,:),'b')
% plot(boundKinemMap.t(1:end), boundKinemMap.l5(3,:),'g')


figure(4)
clf
plot(boundKinemMap.t(1:end-1), boundKinemMap.l1d(1,:),'r')
hold on;
plot(boundKinemMap.t(1:end-1), boundKinemMap.l1d(2,:),'r')
plot(boundKinemMap.t(1:end-1), boundKinemMap.l1d(3,:),'r')
plot(boundKinemMap.t(1:end-1), boundKinemMap.l3d(1,:),'g')
plot(boundKinemMap.t(1:end-1), boundKinemMap.l3d(2,:),'g')
plot(boundKinemMap.t(1:end-1), boundKinemMap.l3d(3,:),'g')
plot(boundKinemMap.t(1:end-1), boundKinemMap.l5d(1,:),'b')
plot(boundKinemMap.t(1:end-1), boundKinemMap.l5d(2,:),'b')
plot(boundKinemMap.t(1:end-1), boundKinemMap.l5d(3,:),'b')

figure(5)
clf
plot(boundKinemMap.t(1:end-2), boundKinemMap.l1dd(1,:),'r')
hold on;
plot(boundKinemMap.t(1:end-2), boundKinemMap.l1dd(2,:),'r')
plot(boundKinemMap.t(1:end-2), boundKinemMap.l1dd(3,:),'r')
plot(boundKinemMap.t(1:end-2), boundKinemMap.l3dd(1,:),'g')
plot(boundKinemMap.t(1:end-2), boundKinemMap.l3dd(2,:),'g')
plot(boundKinemMap.t(1:end-2), boundKinemMap.l3dd(3,:),'g')
plot(boundKinemMap.t(1:end-2), boundKinemMap.l5dd(1,:),'b')
plot(boundKinemMap.t(1:end-2), boundKinemMap.l5dd(2,:),'b')
plot(boundKinemMap.t(1:end-2), boundKinemMap.l5dd(3,:),'b')

% figure(6)
% plot(boundKinemMap.t(1:end), boundKinemMap.l2(1,:),'r')
% hold on;
% plot(boundKinemMap.t(1:end), boundKinemMap.l2(2,:),'b')
% plot(boundKinemMap.t(1:end), boundKinemMap.l2(3,:),'g')
% plot(boundKinemMap.t(1:end), boundKinemMap.l4(1,:),'r')
% plot(boundKinemMap.t(1:end), boundKinemMap.l4(2,:),'b')
% plot(boundKinemMap.t(1:end), boundKinemMap.l4(3,:),'g')
% plot(boundKinemMap.t(1:end), boundKinemMap.l6(1,:),'r')
% plot(boundKinemMap.t(1:end), boundKinemMap.l6(2,:),'b')
% plot(boundKinemMap.t(1:end), boundKinemMap.l6(3,:),'g')
% 
% 
figure(7)
clf
plot(boundKinemMap.t(1:end-1), boundKinemMap.l2d(1,:),'m')
hold on;
plot(boundKinemMap.t(1:end-1), boundKinemMap.l2d(2,:),'m')
plot(boundKinemMap.t(1:end-1), boundKinemMap.l2d(3,:),'m')
plot(boundKinemMap.t(1:end-1), boundKinemMap.l4d(1,:),'k')
plot(boundKinemMap.t(1:end-1), boundKinemMap.l4d(2,:),'k')
plot(boundKinemMap.t(1:end-1), boundKinemMap.l4d(3,:),'k')
plot(boundKinemMap.t(1:end-1), boundKinemMap.l6d(1,:),'c')
plot(boundKinemMap.t(1:end-1), boundKinemMap.l6d(2,:),'c')
plot(boundKinemMap.t(1:end-1), boundKinemMap.l6d(3,:),'c')

figure(8)
clf
plot(boundKinemMap.t(1:end-2), boundKinemMap.l2dd(1,:),'m')
hold on;
plot(boundKinemMap.t(1:end-2), boundKinemMap.l2dd(2,:),'m')
plot(boundKinemMap.t(1:end-2), boundKinemMap.l2dd(3,:),'m')
plot(boundKinemMap.t(1:end-2), boundKinemMap.l4dd(1,:),'k')
plot(boundKinemMap.t(1:end-2), boundKinemMap.l4dd(2,:),'k')
plot(boundKinemMap.t(1:end-2), boundKinemMap.l4dd(3,:),'k')
plot(boundKinemMap.t(1:end-2), boundKinemMap.l6dd(1,:),'c')
plot(boundKinemMap.t(1:end-2), boundKinemMap.l6dd(2,:),'c')
plot(boundKinemMap.t(1:end-2), boundKinemMap.l6dd(3,:),'c')

end