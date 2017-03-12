function attitude = compAttitude( completePath, t )
%Attitude is given back in radians
    xdot = solvePoly(completePath.xParams,t,2);
    ydot = solvePoly(completePath.yParams,t,2);
    attitude = atan2(ydot,xdot);
end