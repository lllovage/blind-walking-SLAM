function [geomPlan] = tripodCycle (polygonSeries, heights)
    % Gait planning for two consecutive support patterns given as a
    % PolygonSeries
    % -> If mod (numberPatterns,2) ~= 0 then the function plans the last 
    % pattern as well.
    % -> If a polygonSeries with more than 2 patterns is given as input,
    % the function returns the
    % plan for the whole series, and the patterns not corresponding to a 
    % mod (numberPatterns,2) ~= 0 condition are planned also, in this case
    % the cycle is not completed but the movement is planned.
    % If two consecutive polygons do not intersect, then intermediate steps
    % are proposed for feasibility (this is a tripod gait), in this case 
    % the extra steps added are marked in the output plan.
    
    % heights: At each stage given by polygonSeries, the user can choose a
    % desired height from the body of the robot to its feet, if a scalar is
    % passed here, the height is conserved constant along the whole plan.
    % If a vector of the size of patterns passed in the polygonSeries
    % structure with the desired height values is passed, then it is
    % respected. If wrong height assignments are passed (sizewise) then
    % a default heights vector is proposed.
    
    % Roll and pitch angles are nullified at every phase of the plan in
    % order to command a more stable gait.
    geomPlan = [];
    poly1 = [];
    poly2 = [];
    [stPolygons, swPolygons] = polygonSeries2plainPolygons (polygonSeries);
    epsilon = 0.15; % TO TUNE: In meters----------------------------------
    
    % Important TUNE this epsilon parameter (clearance before arrival
    % of the foot to the final ground
    % position).--------------------------------------------------
    addition = zeros(4,1);
    addition(3,:) = epsilon;
    
    %Prepare heights vector
    if isscalar(heights)
        heights = repmat(heights,size(polygonSeries,2),1);%---------------
    elseif size(heights,1) == 12 && size(heights,2) == 1
        %-----
    else 
        warning('tripodCycle: Bad dimensiosn passed in heights vector, default constant height is forced in the computation.')
        heights = 0.5; %--------------------------------------------------
        heights = repmat(heights,size(polygonSeries,2),1);
    end
   
    
    for i=1:size(polygonSeries,2)
        % Get intersection of two consecutive polygons in the series
        poly1 = stPolygons(i);
        poly2 = stPolygons(i+1);
        polygons = [poly1; poly2];
        [intPoly, tooFar] = intersectPolygons(polygons);
        % Fill in intrmediate phases. Notice "Number" field allows to see
        % FROM which polygon each of the 5 phases corresponds.
        % 1. First polygon
        temp(1).number = i;
        temp(1).phase = 'p1';
        temp(1).COM = polygonSeries(i).COM;
        temp(1).COMdot = polygonSeries(i).COMdot;
        temp(1).COMddot = polygonSeries(i).COMddot;
        temp(1).stFeet = polygonSeries(i).stFeet;    
        for j=1:size(polygonSeries(i).stFeet,1)
            k = polygonSeries(i).stFeet(j);
            eval(['temp(1).c', num2str(k), '= polygonSeries(i).stCoords(:,find(polygonSeries(i).stFeet == ', num2str(k), '));' ]);
            eval(['temp(1).c', num2str(k) ,'dot = [0 0 0];']);
            % Just for completeness: These eval commands perform the next
            % functions:
            %temp(1).c1 = polygonSeries(i).stCoords(:,find(polygonSeries(i).stFeet == 1));
            %temp(1).c1dot = [0 0 0];
        end       
        temp(1).swFeet = polygonSeries(i).swFeet;
        for j=1:size(polygonSeries(i).swFeet,1)
            k = polygonSeries(i).swFeet(j);
            eval(['temp(1).c', num2str(k), '= polygonSeries(i).swCoords(:,find(polygonSeries(i).swFeet == ', num2str(k), ')) + addition;' ]);
            eval(['temp(1).c', num2str(k) ,'dot = nan(1,3);']);
        end
        temp(1).angles = [polygonSeries(i).att, 0, 0];
        temp(1).height = heights(i);
       
        % 2. Polygon intersection from first polygon
        temp(2).number = i;
        temp(2).phase = 'pInt';
        temp(2).COM = intPoly.centroid(1:2)';
        temp(2).COMdot = nan(1,3);
        temp(2).COMddot = nan(1,3);
        temp(2).stFeet = temp(1).stFeet;
        for j=1:size(polygonSeries(i).stFeet,1)
            k = polygonSeries(i).stFeet(j);
            eval(['temp(2).c', num2str(k), '= temp(1).c',num2str(k),';' ]);
            eval(['temp(2).c', num2str(k), 'dot= temp(1).c',num2str(k),'dot;' ]);
        end       
        temp(2).swFeet = polygonSeries(i).swFeet;
        for j=1:size(polygonSeries(i).swFeet,1)
            k = polygonSeries(i).swFeet(j);
            eval(['temp(2).c', num2str(k), '= polygonSeries(i+1).stCoords(:,find(polygonSeries(i+1).stFeet == ', num2str(k), '))+addition;' ]);
            eval(['temp(2).c', num2str(k), 'dot= temp(1).c',num2str(k),'dot;' ]);
        end       
        temp(2).angles = [polygonSeries(i).att, 0, 0];
        temp(2).height = (heights(1)+heights(2))/2;
        
        %3. Hexagon stance (3 old swing feet down)
        temp(3).number = i;
        temp(3).phase = 'hex';
        temp(3).COM = temp(2).COM;
        temp(3).COMdot = temp(2).COMdot;
        temp(3).COMddot = temp(2).COMddot;
        temp(3).stFeet = [polygonSeries(i).stFeet;polygonSeries(i+1).stFeet];
        for j = 1:size(polygonSeries(i).stFeet,1)
            k = polygonSeries(i).stFeet(j);
            eval(['temp(3).c', num2str(k), '= temp(2).c',num2str(k),';' ]);
            eval(['temp(3).c', num2str(k), 'dot= temp(2).c',num2str(k),'dot;' ]);
        end     
        for j=1:size(polygonSeries(i+1).stFeet,1)
            k = polygonSeries(i+1).stFeet(j);
            eval(['temp(3).c', num2str(k), '= polygonSeries(i+1).stCoords(:,find(polygonSeries(i+1).stFeet == ', num2str(k), '));' ]);
            eval(['temp(3).c', num2str(k), 'dot= [0 0 0];']);
        end
        temp(3).swFeet = [];
        temp(3).angles = temp(2).angles;
        temp(3).height = temp(2).height;
        
        %4. Intersection polygon to second polygon (3 new swing feet up)
        temp(4).number = i;
        temp(4).phase = 'pInt3up';
        temp(4).COM = temp(3).COM;
        temp(4).COMdot = temp(3).COMdot;
        temp(4).COMddot = temp(3).COMddot;
        temp(4).stFeet = polygonSeries(i+1).stFeet;
        for j = 1:size(polygonSeries(i+1).stFeet,1)
            k = polygonSeries(i+1).stFeet(j);
            eval(['temp(4).c', num2str(k), '= temp(3).c',num2str(k),';' ]);
            eval(['temp(4).c', num2str(k), 'dot= temp(3).c',num2str(k),'dot;' ]);
        end   
        temp(4).swFeet = polygonSeries(i+1).swFeet;
        for j=1:size(polygonSeries(i+1).swFeet,1)
            k = polygonSeries(i+1).swFeet(j);
            eval(['temp(4).c', num2str(k), '= polygonSeries(i+1).swCoords(:,find(polygonSeries(i+1).swFeet == ', num2str(k), '))+addition;' ]);
            eval(['temp(4).c', num2str(k), 'dot= nan(1,3);']);
        end        
        temp(4).angles = temp(3).angles;
        temp(4).height = temp(3).height;
        
        %5. Second polygon reached
        temp(5).number = i;
        temp(5).phase = 'p2';
        temp(5).COM = polygonSeries(i+1).COM;
        temp(5).COMdot = polygonSeries(i+1).COMdot;
        temp(5).COMddot = polygonSeries(i+1).COMddot;
        temp(5).stFeet = temp(4).stFeet;
        for j=1:size(polygonSeries(i+1).stFeet,1)
            k = polygonSeries(i+1).stFeet(j);
            eval(['temp(5).c', num2str(k), '= temp(4).c',num2str(k),';' ]);
            eval(['temp(5).c', num2str(k), 'dot= temp(4).c',num2str(k),'dot;' ]);
        end     
        for j=1:size(polygonSeries(i+1).swFeet,1)
            k = polygonSeries(i+1).swFeet(j);
            eval(['temp(5).c', num2str(k), '= temp(4).c',num2str(k),';']);
            eval(['temp(5).c', num2str(k), 'dot= temp(4).c',num2str(k),'dot;']);
        end
        temp(5).swFeet = temp(4).swFeet;
        temp(5).angles = [polygonSeries(i+1).att, 0, 0];
        temp(5).height = heights(i+1);       
        geomPlan = [geomPlan, temp];
    end
end