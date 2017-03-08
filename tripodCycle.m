function [output] = tripodCycle (polygonSeries, heights)
    % Gait planning for three consecutive support patterns given as a
    % PolygonSeries
    % -> If mod (numberPatterns,3) ~= 0 then the function plans the last 
    % pattern as well.
    % -> If a polygonSeries with more than 3 patterns is given as input,
    % the function returns the
    % plan for the whole series, and the patterns not corresponding to a 
    % mod (numberPatterns,3) ~= 0 condition are planned also, in this case
    % the cycle is not completed but the movement is planned.
    % If two consecutive polygons do not intersect, then intermediate steps
    % are proposed for feasibility (this is a tripod gait), in this case 
    % the extra steps added are marked in the output plan.
    
    % heights: At each stage given by polygonSeries, the user can choose a
    % desired height from the body of the robot to its feet, if a scalar is
    % passed here, the height is conserved constant along the whole plan. 
    
    % Roll and pitch angles are nullified at every phase of the plan in
    % order to command a more stable gait.
    geomPlan = [];
    poly1 = [];
    poly2 = [];
    [stPolygons, swPolygons] = polygonSeries2plainPolygons (polygonSeries);
    epsilon = 0.15; % TO TUNE: In meters
    
    for i=1:size(polygonSeries,2)
        poly1 = stPolygons(i);
        poly2 = stPolygons(i+1);
        polygons = [poly1; poly2];
        [intPoly, tooFar] = intersectPolygons(polygons);
        % 1. First polygon
        temp(1).phase = 'p1';
        temp(1).COM = polygonSeries(i).COM;
        temp(1).COMdot = polygonSeries(i).COMdot;
        temp(1).COMddot = polygonSeries(i).COMddot;
        temp(1).stFeet = polygonSeries(i).stFeet;
        
        for i=1:size(polygonSeries(i).stFeet,1)
            j = polygonSeries(i).stFeet(i);
            eval(['temp(1).c', num2str(j), '= polygonSeries(i).stCoords(:,find(polygonSeries(i).stFeet == ', num2str(j), '));' ]);
            %temp(1).c1 = polygonSeries(i).stCoords(:,find(polygonSeries(i).stFeet == 1));
n        end
        temp(1).swFeet = polygonSeries(i).swFeet;
        temp(1).angles = [polygonSeries(i).att, 0, 0];
        % 2. Polygon intersection between first and second polygon
        temp(2).phase = 'pInt';
        temp(2).COM = intPoly.centroid(1:2)';
        temp(2).COMdot = nan(1,3);
        temp(2).COMddot = nan(1,3);
        temp(2).stFeet = temp(1).stFeet;
        temp(2).stCoords = temp(1).stCoords;
        temp(2).stVels = temp(1).stVels;
        temp(2).swFeet = polygonSeries(i).swFeet;
            % Important TUNE this epsilon parameter (clearance before arrival
            % of the foot to the final ground position).
            addition = zeros(4,size(polygonSeries(i+1).stCoords,2));
            addition(3,:) = epsilon;
        temp(2).swCoords = polygonSeries(i+1).stCoords + addition;
        %-----------
        temp(2).swVels = nan(1,3);
        temp(2).angles = [polygonSeries(i).att, 0, 0];
        %3. Hexagon stance (all feet on ground)
        temp(3).phase = 'hex';
        temp(3).COM = polygonSeries(i).COM;
        temp(3).COMdot = polygonSeries(i).COMdot;
        temp(3).COMddot = polygonSeries(i).COMddot;
        temp(3).stFeet = polygonSeries(i).stFeet;
        temp(3).stCoords = polygonSeries(i).stCoords;
        temp(3).stVels = [0,0,0];
        temp(3).swFeet = polygonSeries(i).swFeet;
        temp(3).swCoords = polygonSeries(i).swCoords;
        temp(3).swVels = nan(1,3);
        temp(3).angles = [polygonSeries(i).att, 0, 0];
        
        geomPlan = [geomPlan, temp];
    end
end