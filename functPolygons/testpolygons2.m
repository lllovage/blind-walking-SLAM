% Define first polygon (triangle)
polygon1.feet = [1;2;3];
polygon1.coords = [2,2;5,2;2,4];
polygon1.A = areaPolygon2D ( polygon1 );
[polygon1.cent(1,1), polygon1.cent(2,1), polygon1.cent(3,1),polygon1.cent(4,1)] = centroidPolygon2D ( polygon1 );
% Define second polygon (triangle)
polygon2.feet = [1;2;3];
polygon2.coords = [3,3;4,1;4,4];
polygon2.A = areaPolygon2D ( polygon2 );
[polygon2.cent(1,1), polygon2.cent(2,1), polygon2.cent(3,1),polygon2.cent(4,1)] = centroidPolygon2D ( polygon2 );
% Merge polygons
polygons = [polygon1; polygon2];
% Obtain intersection
intPolygon = intersectPolygons(polygons);
%% EXPERIMENT 3
clc; clear;
% Define first polygon (triangle)
polygon1.feet = [1;2;3;4];
polygon1.coords = [1,2;6,2;6,5;1,5];
polygon1.A = areaPolygon2D ( polygon1 );
[polygon1.cent(1,1), polygon1.cent(2,1), polygon1.cent(3,1),polygon1.cent(4,1)] = centroidPolygon2D ( polygon1 );
% Define second polygon (triangle)
polygon2.feet = [1;2;3];
polygon2.coords = [5,4;7,4;6,6];
polygon2.A = areaPolygon2D ( polygon2 );
[polygon2.cent(1,1), polygon2.cent(2,1), polygon2.cent(3,1),polygon2.cent(4,1)] = centroidPolygon2D ( polygon2 );
% Merge polygons
polygons = [polygon1; polygon2];
% Obtain intersection
intPolygon = intersectPolygons(polygons);
%% EXPERIMENT 4
clc; clear;
% Define first polygon (triangle)
polygon1.feet = [1;2;3;4;5;6];
polygon1.coords = [3,3;4,3;5,4;4,5;3,5;2,4];
polygon1.A = areaPolygon2D ( polygon1 );
[polygon1.cent(1,1), polygon1.cent(2,1), polygon1.cent(3,1),polygon1.cent(4,1)] = centroidPolygon2D ( polygon1 );
% Define second polygon (triangle)
polygon2.feet = [1;2;3;4;5;6];
polygon2.coords = [4,3;5,3;6,4;5,5;4,5;3,4];
polygon2.A = areaPolygon2D ( polygon2 );
[polygon2.cent(1,1), polygon2.cent(2,1), polygon2.cent(3,1),polygon2.cent(4,1)] = centroidPolygon2D ( polygon2 );
% Merge polygons
polygons = [polygon1; polygon2];
% Obtain intersection
intPolygon = intersectPolygons(polygons);
%% EXPERIMENT 5
clc; clear;
% Define first polygon (triangle)
polygon1.feet = [1;2;3;4;5;6];
polygon1.coords = [3,3;4,3;5,4;4,5;3,5;2,4];
polygon1.A = areaPolygon2D ( polygon1 );
[polygon1.cent(1,1), polygon1.cent(2,1), polygon1.cent(3,1),polygon1.cent(4,1)] = centroidPolygon2D ( polygon1 );
% Define second polygon (triangle)
polygon2.feet = [1;2;3;4;5;6];
polygon2.coords = [4,2;5,2;6,3;5,4;4,4;3,3];
polygon2.A = areaPolygon2D ( polygon2 );
[polygon2.cent(1,1), polygon2.cent(2,1), polygon2.cent(3,1),polygon2.cent(4,1)] = centroidPolygon2D ( polygon2 );
% Merge polygons
polygons = [polygon1; polygon2];
% Obtain intersection
intPolygon = intersectPolygons(polygons);
%% EXPERIMENT 6
clc; clear;
% Define first polygon (triangle)
polygon1.feet = [1;2;3;4;5;6];
polygon1.coords = [3,3;4,3;5,4;4,5;3,5;2,4];
polygon1.A = areaPolygon2D ( polygon1 );
[polygon1.cent(1,1), polygon1.cent(2,1), polygon1.cent(3,1),polygon1.cent(4,1)] = centroidPolygon2D ( polygon1 );
% Define second polygon (triangle)
polygon2.feet = [1;2;3];
polygon2.coords = [4,3;6,4;4,5];
polygon2.A = areaPolygon2D ( polygon2 );
[polygon2.cent(1,1), polygon2.cent(2,1), polygon2.cent(3,1),polygon2.cent(4,1)] = centroidPolygon2D ( polygon2 );
% Merge polygons
polygons = [polygon1; polygon2];
% Obtain intersection
intPolygon = intersectPolygons(polygons);