function boundKinemMap = kinemBound (feasKinemMap)
% Find if actuator velocities and accelerations lie inside
% pregiven bounds for OCTOPUS.
% Input: Pass the obtained feasibility Kinematic Map if ssuccessfully
% obtained.
% Output: Structure with a recapitulative of the lengths, velocities and
% accelerations in time as well as the respecitve flags and flag value
% additions for final feasibility summary.

%Determine sizes
samples = [];
ranges = zeros(1,2);
i=1;
cont=1;
while i < size(feasKinemMap,2)
    temp = 6*feasKinemMap(i).samples;
    samples = [samples; temp/6];
    for j=1:6
        ranges = [ranges;[ranges(end,2)+1,ranges(end,2)+temp/6]];
    end
    i=i + temp;
    cont = cont+1;
end  
ranges = ranges(2:end,:);
finalRanges = zeros(size(samples,1),6);
cont=1;
for i=1:size(samples,1)
    for j=1:6
        finalRanges(i,j,1:2) = ranges(cont,:);
        cont = cont+1;
    end   
end
resFinRanges = zeros(size(samples,1),12);
for j=1:6
    resFinRanges(:,2*j-1:2*j) = reshape (finalRanges(:,j,:),size(samples,1),2);
end

leg1 = zeros(3,size(feasKinemMap,2)/6);
leg2 = zeros(3,size(feasKinemMap,2)/6);
leg3 = zeros(3,size(feasKinemMap,2)/6);
leg4 = zeros(3,size(feasKinemMap,2)/6);
leg5 = zeros(3,size(feasKinemMap,2)/6);
leg6 = zeros(3,size(feasKinemMap,2)/6);
for i=1:size(samples,1)
    if i == 1
    temp = 0;  
    else
        temp = temp + samples(i-1);
    end
    t(:,temp+1:temp+samples(i)) = [feasKinemMap(resFinRanges(i,1):resFinRanges(i,2)).t];
    leg1(:,temp+1:temp+samples(i)) = [feasKinemMap(resFinRanges(i,1):resFinRanges(i,2)).l];
    leg2(:,temp+1:temp+samples(i)) = [feasKinemMap(resFinRanges(i,3):resFinRanges(i,4)).l];
    leg3(:,temp+1:temp+samples(i)) = [feasKinemMap(resFinRanges(i,5):resFinRanges(i,6)).l];
    leg4(:,temp+1:temp+samples(i)) = [feasKinemMap(resFinRanges(i,7):resFinRanges(i,8)).l];
    leg5(:,temp+1:temp+samples(i)) = [feasKinemMap(resFinRanges(i,9):resFinRanges(i,10)).l];
    leg6(:,temp+1:temp+samples(i)) = [feasKinemMap(resFinRanges(i,11):resFinRanges(i,12)).l];
end

Ts = feasKinemMap(1).Ts;
leg1d = diff(leg1,1,2)/Ts;
leg2d = diff(leg2,1,2)/Ts;
leg3d = diff(leg3,1,2)/Ts;
leg4d = diff(leg4,1,2)/Ts;
leg5d = diff(leg5,1,2)/Ts;
leg6d = diff(leg6,1,2)/Ts;

leg1dd = diff(leg1d,1,2)/Ts;
leg2dd = diff(leg2d,1,2)/Ts;
leg3dd = diff(leg3d,1,2)/Ts;
leg4dd = diff(leg4d,1,2)/Ts;
leg5dd = diff(leg5d,1,2)/Ts;
leg6dd = diff(leg6d,1,2)/Ts;

flags.Fvel1p = sum(leg1d(1,:)>230)+sum(leg1d(2,:)>230)+sum(leg1d(3,:)>230);
flags.Fvel1m = sum(leg1d(1,:)<-230)+sum(leg1d(2,:)<-230)+sum(leg1d(3,:)<-230);
flags.Fvel2p = sum(leg2d(1,:)>230)+sum(leg2d(2,:)>230)+sum(leg2d(3,:)>230);
flags.Fvel2m = sum(leg2d(1,:)<-230)+sum(leg2d(2,:)<-230)+sum(leg2d(3,:)<-230);
flags.Fvel3p = sum(leg3d(1,:)>230)+sum(leg3d(2,:)>230)+sum(leg3d(3,:)>230);
flags.Fvel3m = sum(leg3d(1,:)<-230)+sum(leg3d(2,:)<-230)+sum(leg3d(3,:)<-230);
flags.Fvel4p = sum(leg4d(1,:)>230)+sum(leg4d(2,:)>230)+sum(leg4d(3,:)>230);
flags.Fvel4m = sum(leg4d(1,:)<-230)+sum(leg4d(2,:)<-230)+sum(leg4d(3,:)<-230);
flags.Fvel5p = sum(leg5d(1,:)>230)+sum(leg5d(2,:)>230)+sum(leg5d(3,:)>230);
flags.Fvel5m = sum(leg5d(1,:)<-230)+sum(leg5d(2,:)<-230)+sum(leg5d(3,:)<-230);
flags.Fvel6p = sum(leg6d(1,:)>230)+sum(leg6d(2,:)>230)+sum(leg6d(3,:)>230);
flags.Fvel6m = sum(leg6d(1,:)<-230)+sum(leg6d(2,:)<-230)+sum(leg6d(3,:)<-230);

flags.Facc1p = sum(leg1dd(1,:)>800)+sum(leg1dd(2,:)>800)+sum(leg1dd(3,:)>800);
flags.Facc1m = sum(leg1dd(1,:)<-800)+sum(leg1dd(2,:)<-800)+sum(leg1dd(3,:)<-800);
flags.Facc2p = sum(leg2dd(1,:)>800)+sum(leg2dd(2,:)>800)+sum(leg2dd(3,:)>800);
flags.Facc2m = sum(leg2dd(1,:)<-800)+sum(leg2dd(2,:)<-800)+sum(leg2dd(3,:)<-800);
flags.Facc3p = sum(leg3dd(1,:)>800)+sum(leg3dd(2,:)>800)+sum(leg3dd(3,:)>800);
flags.Facc3m = sum(leg3dd(1,:)<-800)+sum(leg3dd(2,:)<-800)+sum(leg3dd(3,:)<-800);
flags.Facc4p = sum(leg4dd(1,:)>800)+sum(leg4dd(2,:)>800)+sum(leg4dd(3,:)>800);
flags.Facc4m = sum(leg4dd(1,:)<-800)+sum(leg4dd(2,:)<-800)+sum(leg4dd(3,:)<-800);
flags.Facc5p = sum(leg5dd(1,:)>800)+sum(leg5dd(2,:)>800)+sum(leg5dd(3,:)>800);
flags.Facc5m = sum(leg5dd(1,:)<-800)+sum(leg5dd(2,:)<-800)+sum(leg5dd(3,:)<-800);
flags.Facc6p = sum(leg6dd(1,:)>800)+sum(leg6dd(2,:)>800)+sum(leg6dd(3,:)>800);
flags.Facc6m = sum(leg6dd(1,:)<-800)+sum(leg6dd(2,:)<-800)+sum(leg6dd(3,:)<-800);

boundKinemMap.t = t;
boundKinemMap.l1 = leg1;
boundKinemMap.l2 = leg2;
boundKinemMap.l3 = leg3;
boundKinemMap.l4 = leg4;
boundKinemMap.l5 = leg5;
boundKinemMap.l6 = leg6;
boundKinemMap.l1d = leg1d;
boundKinemMap.l2d = leg2d;
boundKinemMap.l3d = leg3d;
boundKinemMap.l4d = leg4d;
boundKinemMap.l5d = leg5d;
boundKinemMap.l6d = leg6d;
boundKinemMap.l1dd = leg1dd;
boundKinemMap.l2dd = leg2dd;
boundKinemMap.l3dd = leg3dd;
boundKinemMap.l4dd = leg4dd;
boundKinemMap.l5dd = leg5dd;
boundKinemMap.l6dd = leg6dd;
boundKinemMap.flags = flags;
% INOPTIMAL CODE
% for i=1:size(samples,1) % transitions
%     if i == 1
%         kern = 0;
%     else
%         kern = kern + samples(i-1)*6;
%     end
%     
%     if i == 1
%             temp = 0;
%     else temp = temp+samples(i-1);
%     end
%     %-----------------------------
%     for j=1:6 %legs
%         add = samples(i)*j;
%         range1 = temp+1:temp+samples(i);
%         range2 = kern + samples(i)*(j-1) + 1 : kern + add;
%         for k = 1:samples(i)       
%             %boundKinemMap(range1).BjFootPos = feasKinemMap(range2).BFootPos;
%             eval(['boundKinemMap(',num2str(range1(k)),').B',num2str(j),'FootPos = feasKinemMap(',num2str(range2(k)),').BiFootPos;']);
%             %boundKinemMap(range1).FootjPos0 = feasKinemMap(range2).FootPos0;
%             eval(['boundKinemMap(',num2str(range1(k)),').Foot',num2str(j),'Pos = feasKinemMap(',num2str(range2(k)),').FootPos0;']);
%             %boundKinemMap(range1).alphaj = feasKinemMap(range2).alpha;
%             eval(['boundKinemMap(',num2str(range1(k)),').alpha',num2str(j),' = feasKinemMap(',num2str(range2(k)),').alpha;']);
%             %boundKinemMap(range1).betaj = feasKinemMap(range2).beta;
%             eval(['boundKinemMap(',num2str(range1(k)),').beta',num2str(j),' = feasKinemMap(',num2str(range2(k)),').beta;']);
%             %boundKinemMap(range1).lj = feasKinemMap(range2).l;
%             eval(['boundKinemMap(',num2str(range1(k)),').l',num2str(j),' = feasKinemMap(',num2str(range2(k)),').l;']);
%     
%         end
%     end
%     for k = 1:samples(i)
%         boundKinemMap(range1(k)).COM = feasKinemMap(range2(k)).COM;
%         boundKinemMap(range1(k)).t = feasKinemMap(range2(k)).t;
%     end
% end
end
     
