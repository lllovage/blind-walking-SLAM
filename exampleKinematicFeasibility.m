%% Example 1:
clear;clc;
stride = 0.25; % In meters-----------------------------------------------
desiredHeigh = 0.65; % In meters------------------------------------------
Ts = 0.01; %In seconds----------------------------------------------------
timeRes = 0.0001;   % In seconds
% Follow formalism [1 t t^2 t^3 ... t^n]
completePath.xParams = [0.5 0.1]';
completePath.yParams = [0.1 -0.1 -0.1]';
completePath.zParams = [0.65];
completePath.aParams = [0];
completePath.bParams = [0];
completePath.gParams = []; % Following the curve (to be given by input planner)
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
options.default = 1;
% Otherwise need
% options.xParams, options.yParams, options.zParams, options.angles in
% which height will be overriden.
geomPlan = tripodGeometric(polygonSeries,desiredHeigh,options);
geomPlanSimp = simplifyGeomPlan(geomPlan);
showPolygonIntersections (geomPlanSimp);

% Compute feasibility of geometric plan
feasGeomMap = feasibilityGeometric( geomPlanSimp );

%Can use the next commented lines to call planned directly but the
%phasicKinematicPlan function des this directly if a
%feasGeomMap is available.
% constraints(1).time = 0;
% constraints(1).type = 'p';
% constraints(1).value = 0;
% constraints(1).Ts = 0.01;
% %........................
% constraints(2).time = 1;
% constraints(2).type = 'p';
% constraints(2).value = 0.1;
% %........................
% constraints(3).time = 0;
% constraints(3).type = 'v';
% constraints(3).value = 0;
% %........................
% constraints(4).time = 1;
% constraints(4).type = 'v';
% constraints(4).value = 0;
% out = genTraj (constraints);

% Option 1: Plan trajectory in FBreal
kinemPlanBreal = phasicKinematicPlanBreal (feasGeomMap,Ts);
% Option 2: Plan trajectory in F0
kinemPlan = phasicKinematicPlan0 (feasGeomMap,Ts);
% Visualize trajectory planning (ONLY AVAILABLE FOR KINEMATIC PLAN IN
% ABSOLUTE FRAME)
showKinemPlan(geomPlanSimp,kinemPlan0);
%Determine geometrical feasibility for the whole trajectory
feasKinemMap = feasibilityKinematic(kinemPlan0);
showBranchSpeeds();