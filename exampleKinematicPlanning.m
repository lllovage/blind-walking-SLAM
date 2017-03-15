%% Example 1:
clear;clc;
stride = 0.25; % In meters-----------------------------------------------
desiredHeigh = 0.65; % In meters------------------------------------------
timeRes = 0.0001;   % In seconds
completePath.xParams = [0.5 0.1]';
completePath.yParams = [0.1 -0.1 -0.1]';
completePath.t0 = 0; % In seconds
completePath.tf = 5;
slicedPath = slicePath (completePath,stride,timeRes);
showSliced(completePath,slicedPath)

% Fill in slicedPath wth tripods
options.default.ON = 1;
options.default.gait = 'tripod';
options.default.lengthTripod = 1;% Parameter to change-----------------
options.imposeInitialState.ON = 0;
% It is possible to pass an initial correcting state, e.g.:
% options.imposeInitialState.ON = 1;
% options.imposeInitialState.type ='L';
% options.imposeInitialState.stFeet = [1;2;3];
% options.imposeInitialState.swFeet = [4;5;6];
% options.imposeInitialState.COM = [0,0];
% options.imposeInitialState.COMdot = [0,0];
% options.imposeInitialState.COMddot = [0,0];
% options.imposeInitialState.att = 0;
% options.imposeInitialState.stCoords = [[0;-1;0;1],[1;0;0;1],[0;1;0;1]];
% options.imposeInitialState.swCoords = [[0.5;-1;0;1],[1;0.5;0;1],[0.5;1;0;1]];
polygonSeries = polygonSeriesOnPathnew (slicedPath,options);
% Put the vertices in counterclockwise order (this step ensures
% compatibility with polygon functions created).
polygonSeries = reorderPolygonSeries(polygonSeries);
showPolygonSeries(polygonSeries)

% Get complete geometric gait plan
geomPlan = tripodGeometric(polygonSeries,desiredHeigh);
geomPlanSimp = simplifyGeomPlan(geomPlan);
showPolygonIntersections (geomPlanSimp);

% Compute feasibility of geometric plan
feasGeomMap = feasibilityGeometric( geomPlanSimp );

%If geometric plan totally feasible then generate the kinematic plan
