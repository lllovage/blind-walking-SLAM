function T0_Brpy = compT0_Brpy ( COM,angles, height )
% Construct transformation matrix from roll-pitch-yaw body coordinate
% system to the absolute frame 0.
    Rx = compR(angles(1),'x');
    Ry = compR(angles(2),'y');
    Rz = compR(angles(3),'z');    
    T0_Brpy = [[Rz*Ry*Rx;zeros(1,3)],[COM(1);COM(2);height;1] ];
end