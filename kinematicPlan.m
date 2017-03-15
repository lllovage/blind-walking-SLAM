function kinemPlan = kinematicPlan (geomPlanSimp, feasGeomMap)
    % This function take the respective paired simplified geometric plan
    % and feasibility geometric map for this plan in order to compute the
    % kinematic plan of the gait.
    
    % IMPORTANT: The kinematic plan is a copy of the geometric plan with
    % the actuator velocities set with the IKM. If some phase is not
    % feasible geometrically, the function returns the respective kinematic
    % plan but generates a warning about this. Please consider revising
    % the geometric constraints for more feasible constraints before continuing
    % with the control plan.
    
    if feasGeomMap(1).feasValue ~= 0
        warning('Kinematic Plan: Kinematic Plan computed. Not recommended to go further, revise position constraints first.');
    else
        
    end
    
    
end