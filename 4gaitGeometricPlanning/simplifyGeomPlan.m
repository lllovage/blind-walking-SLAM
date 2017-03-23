function geomPlanSimp = simplifyGeomPlan( geomPlan )
    % In the tripodGeometric function a redundat geometric plan was
    % returned. The phase 1 (p1) of each step corresponds to the last phase
    % (p2) of the last step. For this just one of each is needed for the
    % planning. This function states as a standard that each state
    % acertains its forst phase (p1) and with this it is automatically
    % consider that the last step reached for sure its last phase (p2).
    geomPlanSimp = [];
    if strcmp(geomPlan(1).gait, 'initializedTripod');
        geomPlanSimp = [geomPlanSimp, geomPlan(1:2)];
        start = 3;
    else start = 1; 
    end
    for i = 1:size(geomPlan(start:end),2)/5
        geomPlanSimp = [geomPlanSimp, geomPlan(5*(i-1)+1+(start-1):5*(i-1)+4+(start-1))];
    end
    if strcmp(geomPlan(1).gait, 'initializedTripod');
        geomPlanSimp = [geomPlanSimp(1:2), geomPlanSimp(4:end)];
    end


end

