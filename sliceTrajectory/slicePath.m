function slicedPath = slicePath (completePath,stride,timeRes)
    %completePath is a series of coefficients representing a polynomial
    %expression in time with time measured in seconds.
    
    %slicedPath returns the path in admissible chunks or strides to be
    %followed by the robot
    
    %stride: Defines the desired advance per cycle of the robot, this must
    %be changed in order to ensure feasibility in conjunction with the
    %geometric model of the robot (in meters).
    
    %The method used is integral along the curve.
    % timeRes: Time scanning: allows to find the via points along the curve with a
    % better or worse accuracy, put a value in seconds here. One must try
    % with several values in resolution (e.g. 0.01 or 0.001) to choose the
    % resolution that one needs.
    
    %Robot is supposed to be located at initial state imposed by input
    %path.
    viaPos = [];
    viaVel = [];
    viaAcc = [];
    viaAtt = [];
    slicedPath.deltaPos = [];
    deltaPos = 0;
    
    x = solvePoly(completePath.xParams,completePath.t0,1);
    y = solvePoly(completePath.yParams,completePath.t0,1);
    t = completePath.t0;
    while 1
        t = t + timeRes;
        if t > completePath.tf
            break
        end
        xnew = solvePoly(completePath.xParams,t,1);
        ynew = solvePoly(completePath.yParams,t,1);
        deltaPos = sqrt((xnew-x)^2+(ynew-y)^2);
        if deltaPos >= stride
            % Notice that for function solvePoly, it suffices to pass only
            % the position parameters and the mathematical time derivation
            % is performed inside of the function.
            viaPos = [viaPos; [t, xnew, ynew]];
            % Get velocity constraints
            xnewd = solvePoly(completePath.xParams,t,2);
            ynewd = solvePoly(completePath.yParams,t,2);
            viaVel = [viaVel; [t, xnewd, ynewd]];
            % Get acceleration constraints
            xnewdd = solvePoly(completePath.xParams,t,3);
            ynewdd = solvePoly(completePath.yParams,t,3);
            viaAcc = [viaAcc; [t, xnewdd, ynewdd]];
            % Notice: Atittude given back in radians
            newAttitude = compAttitude( completePath, t );
            viaAtt = [viaAtt; [t,newAttitude]];
            x = xnew;
            y = ynew;
            %Prepare output until now
            slicedPath.pos = viaPos;
            slicedPath.vel = viaVel;
            slicedPath.acc = viaAcc;
            slicedPath.att = viaAtt;
            slicedPath.deltaPos = [slicedPath.deltaPos ;[t, deltaPos]];
            slicedPath.t0 = completePath.t0;
            slicedPath.tf = completePath.tf;
            %Reset values
            deltaPos = 0;
            %viaPos = [];
            %viaVel = [];
            %viaAcc = [];
            %viaAtt = [];
        end 
    end
end