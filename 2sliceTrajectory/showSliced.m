function showSliced(completePath, slicedPath)
%     xend = solvePoly(completePath.xParams,completePath.tf,1);
%     yend = solvePoly(completePath.yParams,completePath.tf,1);
%     x = 0:0.1:xend;
%     y = 0:0.1:yend;
    t = completePath.t0:0.01:completePath.tf;
    t = t';
    x = zeros(size(t,1),1);
    y = zeros(size(t,1),1);
    for i=1:size(t,1)
        x(i) = solvePoly(completePath.xParams, t(i),1);
        y(i) = solvePoly(completePath.yParams, t(i),1);
    end
    
    figure(1);
    clf
    plot(x,y,'b'); hold on;
    plot(slicedPath.pos(:,2),slicedPath.pos(:,3),'rx')
    hold on;
    for i=1:size(slicedPath.att,1)
        T0_Brpy = compT0_Brpy ( [slicedPath.pos(i,2),slicedPath.pos(i,3)],[0,0,slicedPath.att(i,2)], 0.67 );
        vec0 = T0_Brpy*[0.2;0;0;1];
        p0 = [slicedPath.pos(i,2), slicedPath.pos(i,3)];
        p1 = [vec0(1), vec0(2)];
        vectarrow(p0,p1);
    end
    
end