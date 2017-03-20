function kinemPlan = phasicKinematicPlan0 (feasGeomMap,Ts)
    % This function take the respective paired simplified geometric plan
    % and feasibility geometric map for this plan in order to compute the
    % kinematic plan of the gait. The function considers phasic
    % transitions and therefore the initial and final speed between phases
    % are considered equal to zero. For imposing continuity constraints
    % look at continuousKinematicPlan function.
    % Ts: Time resolution of the simulation to generate paths
    
    % TRAJECTORY OBTAINED IN ABSOLUTE FRAME!
    
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
        % Start with trajectories of feet in absolute frame
        for j = 1:6 % legs
          for k= 1:3 % x, y, z
            % Allow only computations in vertical direction if feet have to
            % land or take off. In plane direction impose null movement.
            if strcmp(feasGeomMap((i-1)*6+1).phase,'pInt') && feasGeomMap((i-1)*6+j).st1_sw0 == 0 && k ~= 3   
                t0 = feasGeomMap((i-1)*6+1).t;
                tf = feasGeomMap((i)*6+1).t;
                p0 = feasGeomMap((i-1)*6+j).FootPos0(k);
                pf = feasGeomMap((i)*6+j).FootPos0(k);
                v0 = 0;
                vf = 0;
                traj.posParams = zeros(4,1);
                traj.velParams = zeros(4,1);
                traj.accParams = zeros(4,1);
                traj.sim.t = t0:Ts:tf;
                traj.sim.pos = pf*ones(1,size(traj.sim.t,2));
                traj.sim.vel = zeros(1,size(traj.sim.t,2));
                traj.sim.acc = zeros(1,size(traj.sim.t,2));           
            elseif strcmp(feasGeomMap((i-1)*6+1).phase,'hex') && feasGeomMap((i)*6+j).st1_sw0 == 0 && k ~= 3      
                t0 = feasGeomMap((i-1)*6+1).t;
                tf = feasGeomMap((i)*6+1).t;
                p0 = feasGeomMap((i-1)*6+j).FootPos0(k);
                pf = feasGeomMap((i)*6+j).FootPos0(k);
                v0 = 0;
                vf = 0;
                traj.posParams = zeros(4,1);
                traj.velParams = zeros(4,1);
                traj.accParams = zeros(4,1);
                traj.sim.t = t0:Ts:tf;
                traj.sim.pos = pf*ones(1,size(traj.sim.t,2));
                traj.sim.vel = zeros(1,size(traj.sim.t,2));
                traj.sim.acc = zeros(1,size(traj.sim.t,2));
            else
                t0 = feasGeomMap((i-1)*6+1).t;
                p0 = feasGeomMap((i-1)*6+j).FootPos0(k);
                v0 = 0;
                tf = feasGeomMap((i)*6+1).t;
                pf = feasGeomMap((i)*6+j).FootPos0(k);
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
            end % end land or take off IF
            
            % Prepare output structure
            out((i-1)*6+j).step = feasGeomMap((i-1)*6+j).number;
            out((i-1)*6+j).trans = i;
            out((i-1)*6+j).t0 = t0;
            out((i-1)*6+j).tf = tf;
            out((i-1)*6+j).trans = [feasGeomMap((i-1)*6+1).phase, '_', feasGeomMap((i)*6+1).phase];             
            out((i-1)*6+j).leg = j;
            if feasGeomMap((i-1)*6+j).st1_sw0 == 0 && feasGeomMap((i)*6+j).st1_sw0 == 1
                out((i-1)*6+j).Sw2St = 1;
                out((i-1)*6+j).St2Sw = 0;
            elseif feasGeomMap((i-1)*6+j).st1_sw0 == 1 && feasGeomMap((i)*6+j).st1_sw0 == 0
                out((i-1)*6+j).Sw2St = 0;
                out((i-1)*6+j).St2Sw = 1;
            else
                out((i-1)*6+j).Sw2St = 0;
                out((i-1)*6+j).St2Sw = 0;
            end
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
          end % --- end x,y,z
            out((i-1)*6+j).p0 = feasGeomMap((i-1)*6+j).FootPos0;
            out((i-1)*6+j).pf = feasGeomMap((i)*6+j).FootPos0;
            out((i-1)*6+j).v0 = v0;
            out((i-1)*6+j).vf = vf;
        end %---end legs
        
        % Afterwards, compute COM trajectory
        for ck = 1:3 %x, y, z in COM
            t0 = feasGeomMap((i-1)*6+1).t;
            tf = feasGeomMap((i)*6+1).t;
            p0 = feasGeomMap((i-1)*6+1).COM0(ck);
            pf = feasGeomMap((i)*6+1).COM0(ck);
            v0 = 0;
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
            COMtraj = genTraj (constraints);
            out((i-1)*6+1).COM.COM00(ck) = p0;
            out((i-1)*6+1).COM.COM0f(ck) = pf;
            out((i-1)*6+1).COM.sim.t = COMtraj.sim.t;
            if ck == 1
                out((i-1)*6+1).COM.params.xpos = COMtraj.posParams;
                out((i-1)*6+1).COM.params.xvel = COMtraj.velParams;
                out((i-1)*6+1).COM.params.xacc = COMtraj.accParams;
                out((i-1)*6+1).COM.sim.xpos = COMtraj.sim.pos;
                out((i-1)*6+1).COM.sim.xvel = COMtraj.sim.vel;
                out((i-1)*6+1).COM.sim.xacc = COMtraj.sim.acc;
            elseif ck == 2
                out((i-1)*6+1).COM.params.ypos = COMtraj.posParams;
                out((i-1)*6+1).COM.params.yvel = COMtraj.velParams;
                out((i-1)*6+1).COM.params.yacc = COMtraj.accParams;
                out((i-1)*6+1).COM.sim.ypos = COMtraj.sim.pos;
                out((i-1)*6+1).COM.sim.yvel = COMtraj.sim.vel;
                out((i-1)*6+1).COM.sim.yacc = COMtraj.sim.acc;
            elseif ck == 3
                out((i-1)*6+1).COM.params.zpos = COMtraj.posParams;
                out((i-1)*6+1).COM.params.zvel = COMtraj.velParams;
                out((i-1)*6+1).COM.params.zacc = COMtraj.accParams;
                out((i-1)*6+1).COM.sim.zpos = COMtraj.sim.pos;
                out((i-1)*6+1).COM.sim.zvel = COMtraj.sim.vel;
                out((i-1)*6+1).COM.sim.zacc = COMtraj.sim.acc;
            else
            end
        end % end COM trajectory computation      
    end % ---end transitions
    out(1).Ts = Ts;
    kinemPlan = out;
    kinemPlan(1).gait = feasGeomMap(1).gait;
end