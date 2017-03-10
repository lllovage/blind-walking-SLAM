function geomPlanSimp = simplifyGeomPlan( geomPlan )
    % In the tripodGeometric function a redundat geometric plan was
    % returned. The phase 1 (p1) of each step corresponds to the last phase
    % (p2) of the last step. For this just one of each is needed for the
    % planning. This function states as a standard that each state
    % acertains its forst phase (p1) and with this it is automatically
    % consider that the last step reached for sure its last phase (p2).
    geomPlanSimp = [];
    for i = 1:size(geomPlan,2)/5
        geomPlanSimp = [geomPlanSimp, geomPlan(5*(i-1)+1:5*(i-1)+4)];
    end


end

