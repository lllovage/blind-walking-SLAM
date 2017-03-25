function [m,b,flag,xinf] = lineEquation( fi,fi1 )
% Flag = 1 if m=inf, 0 otherwise
    epsilon = 0.001; % 1 mm resolution at singularity
    x1 = fi(1);
    x2 = fi1(1);
    y1 = fi(2);
    y2 = fi1(2);
    if (x2) > ((x1)+ epsilon) || (x2) < ((x1)- epsilon)
        m = (y2-y1)/(x2-x1);
        b = -m*x1 + y1;
        flag = 0;
        xinf = 0;
    else
        m = 0; b = 0; flag = 1; xinf = (x1+x2)/2;
    end
end