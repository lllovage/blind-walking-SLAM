function polygonSeries = polygonSeriesOnPath (slicedPath,options)
    % This function overlaps polygons along the via points on a slicedPath,
    % the options modify the behavior of the function.
    
    % If Default mode is desired in general (initial and all feet poses
    % will be imposed by a default algorithm then it has to be told this
    % algorithm by passing in the structure the next command).
    % struct.default.ON = 1 if default desired, 0 if not. 
    % struct.default.gait = 'tripod' or 'FTL' depending on the desired
    % planning.
    % struct.default.lengthTripod is a value in meters that imposes in
    % the case of tripod gait how big the length of the side of the
    % triangles should be.
    % Notice that the default algorithm considers even terrain and uses
    % 'SM' stability measure for planning.
    
    % Notice that if not
    % desired, then the supplementary functions have to be ON to modify the
    % gait planning, otherwise default planning will end up being used. One
    % can modify the initial condition also with default planning.
   
    % Supplementary Options: MUST be a structure in the next layout:
    % struct.imposeInitialState.ON = 1 or 0. If 1, initial state imposed,
    % otherwise a default is used.
    % with next options:
    % struct.imposeInitialState.COM = [x,y]
    % struct.imposeInitialState.stFeet = [1;2; ...; i] (put number of feet)
    % struct.imposeInitialState.stCoords = [x1,y1; x2,y2; ...;xi,yi]
    % struct.imposeInitialState.swFeet = [1;2; ...; i] (put number of feet)
    % struct.imposeInitialState.swCoords = [x1,y1; x2,y2; ...;xi,yi]
    % struct.imposeInitialState.COM = [x,y]
    % struct.imposeInitialState.COMdot = [xdot,ydot]
    % struct.imposeInitialState.COMddot = [xddot,yddot]
    % struct.imposeInitialState.att = a -> Value in radians measured
    % from the inertial frame in which everything is being planned.
    % ONLY USE THIS OPTION FOR FINITELY SMALL CORRECTIONS FROM THE INITIAL
    % STATE IMPOSED DURING PATH PLANNING. Notice that for stCoords and
    % swCoords one must give a 4 x ni matrix with ni the number of feet in
    % stance or swing phase respectively. The 4 coordinates correspond to
    % x,y,0,1 (zero in z coordinate and 1 in last one corresponding to
    % scale in homogeneous coordinates).
    % 
    % struct.terrainOnline.ON = 1 or 0. If 1, then the function returns
    % only the polygon at the next position on the path then the function
    % should be used in a loop in which feasible terrain measurements are arriving
    % constantly, e.g. via Kinect along with values of attitude and [vel,
    % acc]COM.
    % Notice that for this the input must include the stance feet numbers and stance feet
    % coordinates in the next state as e.g. with a Kinect could be done. 
    % Use the structure options as this:
    % struct.terrainOnline.nextStanceFeet = [1, 2,..., i] (put number of feet)
    % struct.terrainOnline.nextStanceCoords = [x1,y1; x2,y2; ...;xi,yi]
    % struct.terrainOnline.nextAttitude = a -> Value in radians measured
    % from the inertial frame in which everything is being planned.
    % Checking functions are used to achieve feasibility (e.g. stance feet
    % now cannot be stance feet afterwards in another position but swing
    % feet can.)
    % FOR UNEVEN TERRAIN: Consider this function plans only the x,y feet
    % positioning, to include the z positioning then the specific cycle
    % function (not this function)
    % either tripod or FTL cycle has to be used: these functions consider
    % the third component of the position of the legs in order to plan the
    % motion of the legs to go from one of the states to the next one. This
    % function only imposes z = 0 for all feet (even terrain like).

    if options.default.ON == 1
        if strcmp(options.default.gait, 'tripod')
            % Pass a slicedPath to the tripod positioning function
            tripodPath = tripodFill (slicedPath, options.default.lengthTripod);
            % Check if FINITELY different initial state is desired
            if options.imposeInitialState.ON == 1
                tempInitialState(1).type = tripodPath(2).type;
                tempInitialState(1).stFeet = options.imposeInitialState.stFeet;
                tempInitialState(1).swFeet = options.imposeInitialState.swFeet;
                tempInitialState(1).COM = options.imposeInitialState.COM;
                tempInitialState(1).COMdot = options.imposeInitialState.COMdot;
                tempInitialState(1).COMddot = options.imposeInitialState.COMddot;
                tempInitialState(1).att = option.imposeInitialState.att;
                tempInitialState(1).stCoords = options.imposeInitialState.stCoords;
                tempInitialState(1).swCoords = options.imposeInitialState.swFeet;
                tripodPath = [tempInitialState,tripodPath];
            else
                %---- no action
            end
            polygonSeries = tripodPath;
        elseif strcmp(options.default.gait, 'FTL')
            %-----
        end
    end
    
    if options.default.ON == 0
        %---
    end
end