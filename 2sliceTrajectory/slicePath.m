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
    
    % THIS FUNCTION GIVES THE ATTITUDE ALONG THE PATH, THIS ATTITUDE IS THE
    % ANGLE IN ABSOLUTE FRAME OF THE VECTOR PERPENDICULAR TO THE PLANAR
    % TRAJECTORY.
    
    viaPos = zeros(1000,4);
    viaVel = zeros(1000,4);
    viaAcc = zeros(1000,4);
    viaAtt = zeros(1000,2);
    viaDeltaPos = zeros(1000,1);

    slicedPath.deltaPos = [];
    deltaPos = 0;
    
    [x,tempParams] = solvePoly(completePath.xParams,completePath.t0,1);
    [y,tempParams] = solvePoly(completePath.yParams,completePath.t0,1);
    zOut = zTrajectory(completePath.t0);
    z = zOut.pos;
    
    t = completePath.t0;
    
    % First posture in sliced path will be initial posture for sure
    [xnew, tempParams] = solvePoly(completePath.xParams,t,1);
    [ynew, tempParams] = solvePoly(completePath.yParams,t,1);
    znew = zOut.pos;
    viaPos(1,:) = [t, xnew, ynew, znew];
    % Get velocity constraints
    [xnewd, tempParams] = solvePoly(completePath.xParams,t,2);
    [ynewd, tempParams] = solvePoly(completePath.yParams,t,2);
    znewd = zOut.vel;
    viaVel(1,:) = [t, xnewd, ynewd, znewd];
    % Get acceleration constraints
    [xnewdd, tempParams] = solvePoly(completePath.xParams,t,3);
    [ynewdd, tempParams] = solvePoly(completePath.yParams,t,3);
    znewdd = zOut.acc;
    viaAcc(1,:) = [t, xnewdd, ynewdd, znewdd];
    % Notice: Atittude given back in radians
    newAttitude = compAttitude( completePath, t )-pi/2;
    viaAtt(1,:) = [t,newAttitude];
    count = 1;
    
    while 1
        t = t + timeRes;
        if t > completePath.tf
            break
        end
        [xnew, tempParams] = solvePoly(completePath.xParams,t,1);
        [ynew, tempParams] = solvePoly(completePath.yParams,t,1);
        zOut = zTrajectory(t);
        znew = zOut.pos;
        znewd = zOut.vel;
        znewdd = zOut.acc;

        deltaPos = sqrt((xnew-x)^2+(ynew-y)^2+(znew-z)^2);
        if deltaPos >= stride
            % Notice that for function solvePoly, it suffices to pass only
            % the position parameters and the mathematical time derivation
            % is performed inside of the function.
            viaPos(count+1,:) = [t, xnew, ynew, znew];
            % Get velocity constraints
            [xnewd, tempParams] = solvePoly(completePath.xParams,t,2);
            [ynewd, tempParams] = solvePoly(completePath.yParams,t,2);
            viaVel(count+1,:) = [t, xnewd, ynewd, znewd];
            % Get acceleration constraints
            [xnewdd, tempParams] = solvePoly(completePath.xParams,t,3);
            [ynewdd, tempParams] = solvePoly(completePath.yParams,t,3);
            viaAcc(count+1,:) = [t, xnewdd, ynewdd, znewdd];
            % Notice: Atittude given back in radians (PERPENDICULAR TO PATH,
            % therefore pi/2 substracted).
            newAttitude = compAttitude( completePath, t )-pi/2;
            viaAtt(count+1,:) = newAttitude;
            x = xnew;
            y = ynew;
            z = znew;
            viaDeltaPos(count+1,:) = deltaPos;
            %Reset values
            deltaPos = 0;
            %viaPos = [];
            %viaVel = [];
            %viaAcc = [];
            %viaAtt = [];
            count = count+1;
        end 
    end
    viaPos = viaPos(1:count,:);
    viaVel = viaVel(1:count,:);
    viaAcc = viaAcc(1:count,:);
    viaAtt = viaAtt(1:count,:);
    %Prepare output 
            slicedPath.pos = viaPos;
            slicedPath.vel = viaVel;
            slicedPath.acc = viaAcc;
            slicedPath.att = viaAtt;
            slicedPath.deltaPos = viaDeltaPos;
            slicedPath.t0 = completePath.t0;
            slicedPath.tf = completePath.tf;
end