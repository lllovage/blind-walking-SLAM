function [cx, cy, cz, d] = centroidPolygon2D ( polygon )
%[polygon.centroid(1,1), polygon.centroid(2,1), polygon.centroid(3,1),polygon.centroid(4,1)] = centroidPolygon2D ( polygon )
    A =  polygon.A;
    stanceFeet = size(polygon.coords,1);
    x = polygon.coords(:,1);
    y = polygon.coords(:,2);
    cx = 0 ; cy = 0; cz = 0; d = 1;
    for i = 1:stanceFeet
         if i ~= stanceFeet
            cx = cx + (x(i) + x(i+1))*(x(i)*y(i+1)-x(i+1)*y(i));
            cy = cy + (y(i) + y(i+1))*(x(i)*y(i+1)-x(i+1)*y(i));
         else
            cx = cx + (x(i) + x(1))*(x(i)*y(1)-x(1)*y(i));
            cy = cy + (y(i) + y(1))*(x(i)*y(1)-x(1)*y(i));
         end
    end
        cx = cx/(6*A);
        cy = cy/(6*A);
end
    

