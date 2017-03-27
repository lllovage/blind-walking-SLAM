function [geomPlan, polygonSeries] = tripodGeometric (polygonSeries, heights, options)
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
    
    % Options determine the behavior of the function with the change of the
    % next functions:
    % options.default = 0 or 1 (0 if default is OFF and 1 if default is ON)
    % in default mode roll and pitch angles are nullified at every phase of 
    % the plan in order to command a more stable gait. If default mode is not
    % desired then the options structure should also include the initial
    % path which contains the next fields:
    % options.xParams, options.yParams, options.zParams, options.angles,
    % Look that in this case options.zParams overrides heights input and
    % priority is given to the trajectory planned.
    
    % 
    % The output structure geomPlan includes already some velocities for
    % the COM and the stance feet (v=0). This velocities are in ABSOLUTE frame and will
    % be used by the Kinematic planner to calculate the final velocities of
    % the feet with respect to the body.
    
    if options.default == 1
        timeFactor = 4; % The time between phases, ------------------------
        % Time we allow from home to tripod configuration------------------
        timeHomeTripod = 2; 
        initialized = 0; % Flag: 1 initialization ready, 0 otherwise
        geomPlan = [];
        poly1 = [];
        poly2 = [];
        [stPolygons, swPolygons] = polygonSeries2plainPolygons (polygonSeries);
        epsilon = 0.05; % TO TUNE: In meters-------------------------------

        % Important TUNE this epsilon parameter (clearance before arrival
        % of the foot to the final ground
        % position).--------------------------------------------------
        addition = zeros(4,1);
        addition(3,:) = epsilon;
        userHeights = heights;
        %Prepare heights vector
        if isscalar(userHeights)
            heights = repmat(userHeights,size(polygonSeries,2),1);%---------------
        elseif size(userHeights,1) == size(polygonSeries,2) && size(heights,2) == 1
            %-----
        else 
            warning('tripodCycle: Bad dimension passed in heights vector, default constant height is forced in the computation.')
            heights = 0.5; %--------------------------------------------------
            heights = repmat(heights,size(polygonSeries,2),1);
        end
        start = 1;
        % In default mode, check if initial state was imposed, if yes then
        % an intermediate phase to a convenient tripod has to be proposed.
        % In this case the next for cycle starts from the third element
        % and not from the first one.
        if strcmp(polygonSeries(1).gait, 'initializedTripod');
            % Acertain initial state imposed (home position of OPTOPUS).
            %This must be reprogrammed if the home posture is changed from
            %'hex' mode to any other configuration.
            temp(1).t = polygonSeries(1).t;
            temp(1).number = 1;
            temp(1).phase = 'hexInit';
            temp(1).COM = polygonSeries(1).COM;
            temp(1).COMdot = polygonSeries(1).COMdot;
            temp(1).COMddot = polygonSeries(1).COMddot;
            temp(1).stFeet = polygonSeries(1).stFeet;
            for j=1:size(polygonSeries(1).stFeet,1)
                k = polygonSeries(1).stFeet(j);
                eval(['temp(1).c', num2str(k), '= polygonSeries(1).stCoords(:,find(polygonSeries(1).stFeet == ', num2str(k), '));' ]);
                eval(['temp(1).c', num2str(k) ,'dot = [0 0 0];']);
            end  
            temp(1).swFeet = polygonSeries(1).swFeet;
            temp(1).angles = [0, 0, polygonSeries(1).att];
            temp(1).height = 548.90171991133911/1000; % This applies only
            % to current home posture of OCTOPUS, must change if home
            % posture changes in reality!
            temp(2).t = polygonSeries(1).t + timeHomeTripod; % Time to reach tripod configuration from home
            temp(2).number = 1;
            temp(2).phase = 'hexInit3up';
            temp(2).COM = polygonSeries(1).COM;
            temp(2).COMdot = polygonSeries(1).COMdot;
            temp(2).COMddot = polygonSeries(1).COMddot;
            newStCoords = [];
            newSwCoords = [];
            for j=1:size(polygonSeries(2).swFeet,1)
                k = polygonSeries(2).swFeet(j);
                temp(2).stFeet(j,1) = k;
                eval(['temp(2).c', num2str(k), '= polygonSeries(1).stCoords(:,find(polygonSeries(1).stFeet == ', num2str(k), '));' ]);
                eval(['temp(2).c', num2str(k) ,'dot = [0 0 0];']);
                newStCoords = [newStCoords, polygonSeries(1).stCoords(:,find(polygonSeries(1).stFeet == k))];
            end
            
            for j=1:size(polygonSeries(2).stFeet,1)
                k = polygonSeries(2).stFeet(j);
                temp(2).swFeet(j,1) = k;
                eval(['temp(2).c', num2str(k), '= polygonSeries(1).stCoords(:,find(polygonSeries(1).stFeet == ', num2str(k), ')) + addition;' ]);
                eval(['temp(2).c', num2str(k) ,'dot = [0 0 0];']);
                newSwCoords = [newSwCoords, polygonSeries(1).stCoords(:,find(polygonSeries(1).stFeet == k))+ addition];
            end 
            temp(2).angles = [0, 0, polygonSeries(1).att];
            temp(2).height = 548.90171991133911/1000;
            initialized = 1;
            % Fill in new initial state in polygonSeries structure
            newPolygon.t = temp(2).t;
            newPolygon.type = 'R';
            newPolygon.stFeet = temp(2).stFeet;
            newPolygon.swFeet = temp(2).swFeet;
            newPolygon.COM = temp(2).COM;
            newPolygon.COMdot = temp(2).COMdot;
            newPolygon.COMddot = temp(2).COMddot;
            newPolygon.att = temp(2).angles(3);
            newPolygon.stCoords = newStCoords;
            newPolygon.swCoords = newSwCoords;
            newPolygon.gait = [];
            newPolygon = reorderPolygonSeries (newPolygon);
            % Reinitialize instrumental matrices: polygonSeries, polygon
            % extraction and heights
            polygonSeries = [polygonSeries(1), newPolygon, polygonSeries(2:end)];
            [stPolygons, swPolygons] = polygonSeries2plainPolygons (polygonSeries);
            if isscalar(userHeights)
                heights = repmat(userHeights,size(polygonSeries,2),1);%---------------
            elseif size(userHeights,1) == size(polygonSeries,2) && size(heights,2) == 1
            %-----
            else 
                warning('tripodCycle: Bad dimension passed in heights vector, default constant height is forced in the computation.')
                heights = 0.5; %--------------------------------------------------
                heights = repmat(heights,size(polygonSeries,2),1);
            end
            geomPlan = [geomPlan, temp];
        end
        
        % Reinitialize time of the whole series due to initial transition
        % of imposed initialized state.
        if initialized == 1
            initDeltaTime = timeHomeTripod;
            start = 2;
            % Actualize polygonSeries instrumental time
            for i = 3:size(polygonSeries,2)
                %Home position to tripod adaptation
                %time------------------------------------------------------
            polygonSeries(i).t = polygonSeries(i).t + initDeltaTime+(initDeltaTime*4/2);
            end
        else
            initDeltaTime = 0;
            start = 1;
        end
        %Start tripod recursion
        for i=start:size(polygonSeries,2)-1
            
            % Get intersection of two consecutive polygons in the series
            poly1 = stPolygons(i);
            poly2 = stPolygons(i+1);
            polygons = [poly1; poly2];
            [intPoly, tooFar] = intersectPolygons(polygons);
            % Fill in intrmediate phases. Notice "Number" field allows to see
            % FROM which polygon each of the 5 phases corresponds.
            % 1. First polygon
            if initialized == 1 && i == start
                initDeltaTime = timeHomeTripod;
            elseif initialized == 1 && i ~= start
                initDeltaTime = 0;
            elseif initialized == 0 && i == start
                initDeltaTime = 0;
            else
                initDeltaTime = 0;
            end
            
            % Get time fractions between stance changes and advance
            sigma = 1.4;%-------------------------!!!!!!!!!!! tune for accomplishing VEL and ACC constraints
            tz = (polygonSeries(i+1).t -polygonSeries(i).t)/(2*(1+sigma));
            td = sigma*tz;
            temp(1).t = polygonSeries(i).t;
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
            temp(1).angles = [0, 0, polygonSeries(i).att];
            temp(1).height = heights(i);
            % 2. Polygon intersection from first polygon
            temp(2).t = polygonSeries(i).t + td;
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
            temp(2).angles = [0, 0, polygonSeries(i).att];
            temp(2).height = (heights(1)+heights(2))/2;

            %3. Hexagon stance (3 old swing feet down)
            temp(3).t = polygonSeries(i).t + td + tz;
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
            temp(4).t = polygonSeries(i).t + td + tz + tz;
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
                %eval(['temp(4).c', num2str(k), '= polygonSeries(i+1).swCoords(:,find(polygonSeries(i+1).swFeet == ', num2str(k), '))+addition;' ]);
                eval(['temp(4).c', num2str(k), '= temp(3).c',num2str(k),'+addition;' ]);
                eval(['temp(4).c', num2str(k), 'dot= nan(1,3);']);
            end        
            temp(4).angles = temp(3).angles;
            temp(4).height = temp(3).height;

            %5. Second polygon reached
            temp(5).t = polygonSeries(i).t + td + tz + tz + td;
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
                %eval(['temp(5).c', num2str(k), '= temp(4).c',num2str(k),';']);
                eval(['temp(5).c', num2str(k), '= polygonSeries(i+1).swCoords(:,find(polygonSeries(i+1).swFeet == ', num2str(k), '))+addition;' ]);
                eval(['temp(5).c', num2str(k), 'dot= temp(4).c',num2str(k),'dot;']);
            end
            temp(5).swFeet = temp(4).swFeet;
            temp(5).angles = [0, 0, polygonSeries(i+1).att];
            temp(5).height = heights(i+1);
   
            geomPlan = [geomPlan, temp];
            
        end
        if initialized == 1
            geomPlan(1).gait = 'initializedTripod';
            % Take away useless state in the modified plan
            geomPlan(2).number = start;
            geomPlan(2).phase = 'p1';
            geomPlan = [geomPlan(1:2), geomPlan(4:end)];
        else
            geomPlan(1).gait = 'tripod';
        end
    else % IF DEFAULT NOT EQUAL TO 1
        % Then angles and heights are imposed for sure. Check options
        % strcture at the input.
    end% END IF DEFAULT
end