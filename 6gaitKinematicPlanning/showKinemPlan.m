function showKinemPlan(geomPlanSimp,kinemPlan)
    figure(2);
    clf;
    if strcmp(kinemPlan(1).gait,'tripod')
        % Assign each tripod a color
        col = 'rkrkrk';
    elseif strcmp(kinemPlan(1).gait,'initializedTripod')
        col = 'gbgbgb';
    end
    
        %TBrpy_Breal = compTBrpy_Breal;
    for i=1:size(geomPlanSimp,2)-1
        if i > 2
            col = 'rkrkrk';
        end
        % Extract phase from geometric plan (Notice we need lengths in mm!)
        phase1 = geomPlanSimp(i);
        phase2 = geomPlanSimp(i+1);
        %T0_Brpy = compT0_Brpy ( phase1.COM*1000, phase1.angles, phase1.height*1000 );
        % Compute feasibility for each of the 6 legs in each phase
        for j = 1:6
            leg = kinemPlan((i-1)*6+j);
            % Extract actual feet position wrt body B
            %FBreal = [leg.sim.xpos; leg.sim.ypos; leg.sim.zpos;1];
            % Transform into absolute frame
            %T0_Breal = T0_Brpy*TBrpy_Breal;
            %F0 = T0_Breal*FBreal;
            F0 = [leg.sim.xpos; leg.sim.ypos; leg.sim.zpos;ones(1,size(leg.sim.zpos,2))];
            plot3(F0(1,:), F0(2,:), F0(3,:), col(j));
            hold on;
        end
        % Plot COM
         COM = 1000*[kinemPlan((i-1)*6+1).COM.sim.xpos;...
                kinemPlan((i-1)*6+1).COM.sim.ypos;...
                kinemPlan((i-1)*6+1).COM.sim.zpos];
         plot3(COM(1,:), COM(2,:), COM(3,:), 'cx');
         hold on;
        
        % Plot stance feet information
            stFeet1 = phase1.stFeet;
            stFeet2 = phase2.stFeet;
            for k = 1:size(stFeet1,1)
                eval(['plot3(phase1.c', num2str(stFeet1(k)),'(1)*1000,', 'phase1.c', num2str(stFeet1(k)),'(2)*1000,', 'phase1.c', num2str(stFeet1(k)),'(3)*1000,','''gs''',')']);
                hold on;
%                 txt1 = ['\leftarrow ', num2str(stFeet1(k)),'t', num2str(i)];
%                 eval(['x1 = phase1.c', num2str(stFeet1(k)),'(1)*1000;']);
%                 eval(['y1 = phase1.c', num2str(stFeet1(k)),'(2)*1000;']);
%                 eval(['z1 = phase1.c', num2str(stFeet1(k)),'(3)*1000;']);
%                 text(x1,y1,z1,txt1); 
%                 hold on;
            end
%             for k = 1:size(stFeet2,1)
%                 eval(['plot3(phase2.c', num2str(stFeet2(k)),'(1)*1000,', 'phase2.c', num2str(stFeet2(k)),'(2)*1000,', 'phase2.c', num2str(stFeet2(k)),'(3)*1000,','''mo''',')']);
%                 hold on;
%                 txt1 = ['\rightarrow ',num2str(stFeet2(k)),'t2'];
%                 eval(['x1 = phase2.c', num2str(stFeet2(k)),'(1)*1000;']);
%                 eval(['y1 = phase2.c', num2str(stFeet2(k)),'(2)*1000;']);
%                 eval(['z1 = phase2.c', num2str(stFeet2(k)),'(3)*1000;']);
%                 text(x1,y1,z1,txt1); 
%             end
    if strcmp(geomPlanSimp(i).phase,'hexInit') || strcmp(geomPlanSimp(i).phase,'p1')
        T0_Brpy = compT0_Brpy ( [geomPlanSimp(i).COM(1),geomPlanSimp(i).COM(2)],geomPlanSimp(i).angles, geomPlanSimp(i).height);
        vec0 = T0_Brpy*[0.2;0;0;1];
        p0 = [geomPlanSimp(i).COM, geomPlanSimp(i).height];
        p1 = [vec0(1), vec0(2), vec0(3)];
        vectarrow(1000*p0,1000*p1);
    else
    end
    end
    
end