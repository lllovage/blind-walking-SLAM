function showPolygonIntersections( geomPlanSimp )
    intersections = [geomPlanSimp(3:4:end).COM];
    x = intersections(1:2:end);
    y = intersections(2:2:end);
    plot(x,y,'cx');
end