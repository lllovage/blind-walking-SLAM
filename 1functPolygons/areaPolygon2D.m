function A = areaPolygon2D ( polygon )
    % NOTICE: Polygon feet in polygon.feet (stance feet numbers in column vector)
              %Polygon coordinates in polygon.coords (x,y column vectors)
              % The rows of polygon.coords follow counterclockwise order
              % of polygon.feet numbers.
              % polygon.A = areaPolygon2D ( polygon )
    stanceFeet = size(polygon.coords,1);
    A = 0;
    x = polygon.coords(:,1);
    y = polygon.coords(:,2);
    for i = 1:stanceFeet
        if i ~= stanceFeet
            A = A + (x(i)*y(i+1)-x(i+1)*y(i));
        else
            A = A + (x(i)*y(1)-x(1)*y(i));
        end
    end
    A = A/2;
end