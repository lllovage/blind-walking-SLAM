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
    
    % The feasibility measure is included at one field in the first row of
    % the structure. This will only say how "much" feasible the whole plan
    % is but the idea is to compute a totally feasible plan with feasValue
    % = 0.
    TBrpy_Breal = compTBrpy_Breal;
    for i=1:size(geomPlanSimp,2)
        % Extract phase from geometric plan (Notice we need lengths in mm!)
        phase = geomPlanSimp(i);
        T0_Brpy = compT0_Brpy ( phase.COM*1000, phase.angles, phase.height*1000 );
        % Compute feasibility for each of the 6 legs in each phase
        for j = 1:6
            
            eval(['F0 = phase.c', num2str(j),';']);
            % Must be in mm!
            F0(1:3) = 1000*(F0(1:3));
            T0_Breal = T0_Brpy*TBrpy_Breal;
            TBreal_0 = invT (T0_Breal);
            FBreal = TBreal_0*F0;
            feasGeomMap(6*(i-1)+j).t = geomPlanSimp(i).t;
            feasGeomMap(6*(i-1)+j).number = geomPlanSimp(i).number;
            feasGeomMap(6*(i-1)+j).phase = geomPlanSimp(i).phase;
            if sum(phase.stFeet == j) > 0
                feasGeomMap(6*(i-1)+j).st1_sw0 = 1;
            else
                feasGeomMap(6*(i-1)+j).st1_sw0 = 0;
            end
            % Compute IGM to find feasibility
            out  = IGM( FBreal , j );
            feasGeomMap(6*(i-1)+j).leg = j;
            feasGeomMap(6*(i-1)+j).BiFootPos = FBreal;
            feasGeomMap(6*(i-1)+j).FootPos0 = F0;
            feasGeomMap(6*(i-1)+j).alpha = out.alpha;
            feasGeomMap(6*(i-1)+j).beta = out.beta;
            feasGeomMap(6*(i-1)+j).l = out.l;
            feasGeomMap(6*(i-1)+j).Fnl1 = out.feasFlags.Fnl1;
            feasGeomMap(6*(i-1)+j).Fnb1 = out.feasFlags.Fnb1;
            feasGeomMap(6*(i-1)+j).Fl1 = out.feasFlags.Fl1;
            feasGeomMap(6*(i-1)+j).Fl2 = out.feasFlags.Fl2;
            feasGeomMap(6*(i-1)+j).Fl3 = out.feasFlags.Fl3;
            feasGeomMap(6*(i-1)+j).Fa1 = out.feasFlags.Fa1;
            feasGeomMap(6*(i-1)+j).Fa2 = out.feasFlags.Fa2;
            feasGeomMap(6*(i-1)+j).Fa3 = out.feasFlags.Fa3;
            feasGeomMap(6*(i-1)+j).Fb1 = out.feasFlags.Fb1;
            feasGeomMap(6*(i-1)+j).Fb2 = out.feasFlags.Fb2;
            feasGeomMap(6*(i-1)+j).Fb3 = out.feasFlags.Fb3;
        end 
        feasGeomMap(6*(i-1)+1).COM0 = [geomPlanSimp(i).COM,geomPlanSimp(i).height];
    end
    feasGeomMap(1).feasValue = confirmGeomFeasibility( feasGeomMap );
    feasGeomMap(1).gait = geomPlanSimp(1).gait;
end