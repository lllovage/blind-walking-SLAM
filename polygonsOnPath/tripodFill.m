function tripodPath = tripodFill (slicedPath, lengthTripod)
    % Propose base triangles
    % Notice:
    %       o         o
    %       1 -       2
    %       |      -  |
    %       |         |-
    %   o   |         |    o 4
    %   3   |         |   - 
    %       |         |-
    %       |       - |
    %       o -       o
    %       5         6
    height = lengthTripod * cos(30*pi/180);
    r1 = [lengthTripod/2,height/3,0,1]';
    r4 = [0,-2*height/3,0,1]';
    r5 = [-lengthTripod/2,height/3,0,1]';
    R = [r1,r4,r5];
    
    l2 = [lengthTripod/2,-height/3,0,1]';
    l3 = [0,2*height/3,0,1]';
    l6 = [-lengthTripod/2,-height/3,0,1]';
    L = [l2,l3,l6];
    
    for i=1:size(slicedPath.pos,1)
        if mod(i,2) == 0
            tripodPath(i).type = 'L';
            tripodPath(i).stFeet = [2;3;6];
            tripodPath(i).swFeet = [1;4;5];
            tripodPath(i).COM = slicedPath.pos(i,2:3);
            tripodPath(i).COMdot = slicedPath.vel(i,2:3);
            tripodPath(i).COMddot = slicedPath.acc(i,2:3);
            tripodPath(i).att = slicedPath.att(i,2);
            T = compT(slicedPath.pos(i,2:3),slicedPath.att(i,2));
%             tripodPath(i).coords.l2 = T*l2;
%             tripodPath(i).coords.l3 = T*l3;
%             tripodPath(i).coords.l6 = T*l6;
              tripodPath(i).stCoords = T*L;
              tripodPath(i).swCoords = T*R;
        else
            tripodPath(i).type = 'R';
            tripodPath(i).stFeet = [1;4;5];
            tripodPath(i).swFeet = [2;3;6];
            tripodPath(i).COM = slicedPath.pos(i,2:3);
            tripodPath(i).COMdot = slicedPath.vel(i,2:3);
            tripodPath(i).COMddot = slicedPath.acc(i,2:3);
            tripodPath(i).att = slicedPath.att(i,2);
            T = compT(slicedPath.pos(i,2:3),slicedPath.att(i,2));
%             tripodPath(i).coords.r1 = T*r1;
%             tripodPath(i).coords.r4 = T*r4;
%             tripodPath(i).coords.r5 = T*r5;
              tripodPath(i).stCoords = T*R;
              tripodPath(i).swCoords = T*L;
        end
    end
end