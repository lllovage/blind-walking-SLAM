function [stPolygons, swPolygons] = polygonSeries2plainPolygons (polygonSeries)
    stPolygons = []; swPolygons = [];
    for i=1:size(polygonSeries,2)
        %Generate polygons structure for each stance and swing series
        temp.feet = polygonSeries(i).stFeet;
        temp.coords = polygonSeries(i).stCoords(1:2,:)';
        temp.A = areaPolygon2D ( temp );
        [temp.cent(1,1), temp.cent(2,1), temp.cent(3,1),temp.cent(4,1)] = centroidPolygon2D ( temp );
        stPolygons = [stPolygons;temp];
        
        temp.feet = polygonSeries(i).swFeet;
        temp.coords = polygonSeries(i).swCoords(1:2,:)';
        temp.A = areaPolygon2D ( temp );
        swPolygons = [swPolygons;temp];
    end
end