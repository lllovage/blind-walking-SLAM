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
end