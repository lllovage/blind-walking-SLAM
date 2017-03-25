%% Create some trajectory in absolute space
clear;clc;
tVia = 15; % seconds between vias
r = 1.3; % in meters
createTrajectory
plot(xOut.sim.pos,yOut.sim.pos)
print('trajectory','-dpng')
% Slice the trajectory in equally separated chunks (norm2)
stride = 0.24; % In meters-----------------------------------------------
desiredHeigh = 0.67; % In meters------------------------------------------
Ts = 0.01; %In seconds----------------------------------------------------
timeRes = 0.001;   % In seconds
% Follow formalism [1 t t^2 t^3 ... t^n]
completePath.xParams = xOut.posParams;
completePath.yParams = yOut.posParams;
completePath.zParams = [desiredHeigh];
completePath.aParams = [0];
completePath.bParams = [0];
completePath.gParams = []; % Following the curve (to be given by input planner)
completePath.t0 = 0; % In seconds
completePath.tf = tVia*6;
slicedPath = slicePath (completePath,stride,timeRes);
showSliced(completePath,slicedPath)
print('trajectorySliced','-dpng')
%% Fill in slicedPath with tripods
options.default.ON = 1;
options.default.gait = 'tripod';
options.default.lengthTripod = 0.9;% Parameter to change-----------------
options.imposeInitialState.ON = 1;
% It is possible to pass an initial correcting state, e.g.:
radius_Sf = 1503.26481494481730/(2*1000);

options.imposeInitialState.t =0;
options.imposeInitialState.type ='H';
options.imposeInitialState.stFeet = [1;2;3;4;5;6];
options.imposeInitialState.swFeet = [];
options.imposeInitialState.COM = [0,0];
options.imposeInitialState.COMdot = [0,0];
options.imposeInitialState.COMddot = [0,0];
options.imposeInitialState.att = 0;
options.imposeInitialState.stCoords = [[sin(pi/6)*radius_Sf;-548.90171991133911/1000;cos(pi/6)*radius_Sf;1],...
                                       [radius_Sf;-548.90171991133911/1000;0;1],...
                                       [sin(pi/6)*radius_Sf;-548.90171991133911/1000;-cos(pi/6)*radius_Sf;1],...
                                       [-sin(pi/6)*radius_Sf;-548.90171991133911/1000;-cos(pi/6)*radius_Sf;1],...
                                       [-radius_Sf;-548.90171991133911/1000;0;1],...
                                       [-sin(pi/6)*radius_Sf;-548.90171991133911/1000;cos(pi/6)*radius_Sf;1]];
COM0 = options.imposeInitialState.COM;
att0 = options.imposeInitialState.att;
%height0 = 0.606901719910;
height0 = 548.90171991133911/1000;
TBrpy_Breal = compTBrpy_Breal;
T0_Brpy = compT0_Brpy ( COM0, [0 0 att0], height0 );
T0_Breal = T0_Brpy*TBrpy_Breal;
options.imposeInitialState.stCoords = T0_Breal*options.imposeInitialState.stCoords;


options.imposeInitialState.swCoords = [];
polygonSeries = polygonSeriesOnPathnew (slicedPath,options);
%%
% Put the vertices in counterclockwise order (this step ensures
% compatibility with polygon functions created).
polygonSeries = reorderPolygonSeries(polygonSeries);
%showPolygonSeries(polygonSeries)
%% Get complete geometric gait plan
options.default = 1;
% Otherwise need
% options.xParams, options.yParams, options.zParams, options.angles in
% which height will be overriden.
[geomPlan,polygonSeries] = tripodGeometric(polygonSeries,desiredHeigh,options);
showPolygonSeries(polygonSeries)
print('trajectoryPolygons','-dpng')
%%
geomPlanSimp = simplifyGeomPlan(geomPlan);
showPolygonIntersections (geomPlanSimp);
print('trajectoryIntersections','-dpng')
% Compute feasibility of geometric plan
feasGeomMap = feasibilityGeometric( geomPlanSimp );
%% Kinematic plan
% Option 1: Plan trajectory in FBreal
%kinemPlanBreal = phasicKinematicPlanBreal (feasGeomMap,Ts);
% Option 2: Plan trajectory in F0
kinemPlan = phasicKinematicPlan0 (feasGeomMap,Ts);
% Visualize trajectory planning (ONLY AVAILABLE FOR KINEMATIC PLAN IN
% ABSOLUTE FRAME)
showKinemPlan(geomPlanSimp,kinemPlan);
print('kinemPlan','-dpng')
%%
%Determine geometrical feasibility for the whole trajectory
feasKinemMap = feasibilityKinematic(kinemPlan);
boundKinemMap = kinemBound (feasKinemMap);
boundKinemMap = confirmKinemFeasibility( boundKinemMap );
showBranchKinem(boundKinemMap);