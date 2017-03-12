function tripodPath = tripodFillnew (slicedPath, lengthTripod)
    % Propose base triangles: NEW FUNCTION DUE TO RELABELLING OF LEGS
    % Notice:
    %       o         o
    %       1 -       6
    %       |      -  |
    %       |         |-
    %   o   |         |    o 5
    %   2   |         |   - 
    %       |         |-
    %       |       - |
    %       o -       o
    %       3         4
    % RELABELLING:
    % 1 = 1
    % 3 = 2
    % 5 = 3
    % 6 = 4
    % 4 = 5
    % 2 = 6
    height = lengthTripod * cos(30*pi/180);
    r1 = [lengthTripod/2,height/3,0,1]';
    r5 = [0,-2*height/3,0,1]';
    r3 = [-lengthTripod/2,height/3,0,1]';
    R = [r1,r5,r3];
    
    l6 = [lengthTripod/2,-height/3,0,1]';
    l2 = [0,2*height/3,0,1]';
    l4 = [-lengthTripod/2,-height/3,0,1]';
    L = [l6,l2,l4];
    
    for i=1:size(slicedPath.pos,1)
        if mod(i,2) == 0
            tripodPath(i).t = slicedPath.pos(i,1);
            tripodPath(i).type = 'L';
            tripodPath(i).stFeet = [6;2;4];
            tripodPath(i).swFeet = [1;5;3];
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
            tripodPath(i).t = slicedPath.pos(i,1);
            tripodPath(i).type = 'R';
            tripodPath(i).stFeet = [1;5;3];
            tripodPath(i).swFeet = [6;2;4];
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