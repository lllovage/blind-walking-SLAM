function feasGeomMap = feasibilityGeometric( geomPlanSimp )
    % This function gets the feasibility flags of the whole geometric plan
    % with respect to the geometric model of OCTOPUS. If no flag is set
    % after running this function, the intermediate phases of the whole
    % trajectory are feasible geometrically (i.e. within addmissible legs' 
    % workspace and robot's whole workspace) and trajectories might be
    % planned from one point to the next one.
    
    % Either completely feasible or not the respective required lengths of
    % each of the leg branches as well as the respective triplets of cardan
    % angles per leg.
    TBrpy_Breal = compTBrpy_Breal;
    for i=1:size(geomPlanSimp,2)
        % Extract phase from geometric plan
        phase = geomPlanSimp(i);
        T0_Brpy = compT0_Brpy ( phase.COM, phase.angles, phase.height );
        % Compute feasibility for each of the 6 legs in each phase
        for j = 1:6
            eval(['F0 = phase.c', num2str(j)]);
            
        end
        
        FBreal = (T0_Brpy*TBrpy_Breal)\F0;
        
    end
    
    
    
    
    
end