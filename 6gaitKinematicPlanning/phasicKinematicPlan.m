function kinemPlan = phasicKinematicPlan (feasGeomMap,Ts)
    % This function take the respective paired simplified geometric plan
    % and feasibility geometric map for this plan in order to compute the
    % kinematic plan of the gait.
    % Ts: Time resolution of the simulation to generate paths
    
    % IMPORTANT: The kinematic plan is a copy of the geometric plan with
    % the actuator velocities set with the IKM. If some phase is not
    % feasible geometrically, the function returns the respective kinematic
    % plan but generates a warning about this. Please consider revising
    % the geometric constraints for more feasible constraints before continuing
    % with the control plan.
    
    if feasGeomMap(1).feasValue ~= 0
        warning('Kinematic Plan: Kinematic Plan of not toally feasible Geometric Plan computed. Not recommended to go further, revise position constraints first.');
    else
    end
    %Prepare constraints
    for i = 1:(size(feasGeomMap,2)/6)-1 %number of full stances minus last one
        for j = 1:6 % legs
            for k= 1:3 % x, y, z
                t0 = feasGeomMap((i-1)*6+1).t;
                p0 = feasGeomMap((i-1)*6+j).BiFootPos(k);
                v0 = 0;
                tf = feasGeomMap((i)*6+1).t;
                pf = feasGeomMap((i)*6+j).BiFootPos(k);
                vf = 0;
                constraints(1).time = t0;
                constraints(1).type = 'p';
                constraints(1).value = p0;
                constraints(1).Ts = Ts;
                %........................
                constraints(2).time = t0;
                constraints(2).type = 'v';
                constraints(2).value = v0;
                %........................
                constraints(3).time = tf;
                constraints(3).type = 'p';
                constraints(3).value = pf;
                %........................
                constraints(4).time = tf;
                constraints(4).type = 'v';
                constraints(4).value = vf;
                % Compute trajectory
                traj = genTraj (constraints);
                % Prepare output structure
                out((i-1)*6+j).t0 = t0;
                out((i-1)*6+j).tf = tf;
                out((i-1)*6+j).trans = [feasGeomMap((i-1)*6+1).phase, '_', feasGeomMap((i)*6+1).phase];             
                out((i-1)*6+j).leg = j;
                if k == 1
                    out((i-1)*6+j).params.xpos = traj.posParams;
                    out((i-1)*6+j).params.xvel = traj.velParams;
                    out((i-1)*6+j).params.xacc = traj.accParams;
                    out((i-1)*6+j).sim.t = traj.sim.t;
                    out((i-1)*6+j).sim.xpos = traj.sim.pos;
                    out((i-1)*6+j).sim.xvel = traj.sim.vel;
                    out((i-1)*6+j).sim.xacc = traj.sim.acc;
                elseif k==2
                    out((i-1)*6+j).params.ypos = traj.posParams;
                    out((i-1)*6+j).params.yvel = traj.velParams;
                    out((i-1)*6+j).params.yacc = traj.accParams;
                    out((i-1)*6+j).sim.t = traj.sim.t;
                    out((i-1)*6+j).sim.ypos = traj.sim.pos;
                    out((i-1)*6+j).sim.yvel = traj.sim.vel;
                    out((i-1)*6+j).sim.yacc = traj.sim.acc;
                elseif k==3
                    out((i-1)*6+j).params.zpos = traj.posParams;
                    out((i-1)*6+j).params.zvel = traj.velParams;
                    out((i-1)*6+j).params.zacc = traj.accParams;
                    out((i-1)*6+j).sim.t = traj.sim.t;
                    out((i-1)*6+j).sim.zpos = traj.sim.pos;
                    out((i-1)*6+j).sim.zvel = traj.sim.vel;
                    out((i-1)*6+j).sim.zacc = traj.sim.acc;
                end
            end
            out((i-1)*6+j).p0 = feasGeomMap((i-1)*6+j).BiFootPos;
            out((i-1)*6+j).pf = feasGeomMap((i)*6+j).BiFootPos;
            out((i-1)*6+j).v0 = v0;
            out((i-1)*6+j).vf = vf;
        end              
    end
    out(1).Ts = Ts;
    kinemPlan = out;
end