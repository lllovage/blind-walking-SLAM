function TBrpy_Breal = compTBrpy_Breal ( )
% Construct transformation matrix from roll-pitch-yaw body coordinate
% system to the actual coordinate system of the robot in NX-UGI.
    Rx = compR(pi/2,'x');
    Rz = compR(pi/2,'z');    
    TBrpy_Breal = [[Rz*Rx;zeros(1,3)],[zeros(3,1);1] ];
end