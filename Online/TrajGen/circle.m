function [x,y,z,a,b,g] = circle(t)
    r = 1; % Radius in meters
    tVia = 60; % Time for the robot to turn one ride;---------------------
    %Position
    x = r + r*cos(2*pi*(1/tVia)*t+pi);
    y = r*sin(2*pi*(1/tVia)*t+pi);
    %Velocity
    xdot = -r*2*pi*(1/tVia)*sin(2*pi*(1/tVia)*t+pi);
    ydot = r*2*pi*(1/tVia)*cos(2*pi*(1/tVia)*t+pi);
    z =  0.65;
    % Orientation roll, pitch, yaw
    a = 0;
    b = 0;
    g = atan2(ydot,xdot);
end