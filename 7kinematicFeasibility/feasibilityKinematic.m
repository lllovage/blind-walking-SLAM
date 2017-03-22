function feasKinemMap = feasibilityKinematic(kinemPlan)
    %This function generates an equivalent geometric plan out of the
    %kinematic plan sent as input. This is performed by unpacking the whole
    %trajectory in the kinematic plan and expressing it discretelike in a
    %geometric plan and then using again geometric feasibility to ensure
    %path connectedness in actuation space.
    
     TBrpy_Breal = compTBrpy_Breal;
     timeTemp = 0;
     timeTemp2 = 0;
     timeStamps = [];
     feasKinemMap(1).samples = size(kinemPlan(1).sim.t,2);
     
     % Determine size of final logging matrix
     samplesTot = 0;
     for i=1:size(kinemPlan,2)/6
         samples = size(kinemPlan(6*(i-1)+1).sim.t,2);
         timeStamps = [timeStamps; samples];
         samplesTot = samplesTot + 6*samples;      
     end

    for i=1:size(kinemPlan,2)/6
        % Compute feasibility for each of the 6 legs at each time instant
        for j = 1:6
            for k = 1:timeStamps(i)
                COM = [kinemPlan(6*(i-1)+1).COM.sim.xpos(k);...
                       kinemPlan(6*(i-1)+1).COM.sim.ypos(k);...
                       kinemPlan(6*(i-1)+1).COM.sim.zpos(k); 1];
              angles = [kinemPlan(6*(i-1)+1).angles.sim.a(k);...
                       kinemPlan(6*(i-1)+1).angles.sim.b(k);...
                       kinemPlan(6*(i-1)+1).angles.sim.g(k)];
                T0_Brpy = compT0_Brpy ( COM(1:2)*1000, angles, COM(3)*1000 );
                posture = kinemPlan(6*(i-1)+j);
                
                F0x = posture.sim.xpos(k);
                F0y = posture.sim.ypos(k);
                F0z = posture.sim.zpos(k);
                F0 = [F0x; F0y; F0z; 1];
        
                % Compute rotation matrix
                T0_Breal = T0_Brpy*TBrpy_Breal;
                TBreal_0 = invT (T0_Breal);
                FBreal = TBreal_0*F0;
                %index = 6*(i-1) + timeStamps*(j-1) + timeTemp + k;
                index = timeTemp + k;
                %Logging
                feasKinemMap(index).t = kinemPlan(6*(i-1)+j).sim.t(k);
                %feasKinemMap(index).tf = kinemPlan(6*(i-1)+j).tf;
                feasKinemMap(index).step = kinemPlan(6*(i-1)+j).step;
                feasKinemMap(index).trans = kinemPlan(6*(i-1)+j).trans;
                if index == (timeTemp2 + 6*timeStamps(i)) && index+1 < samplesTot
                    feasKinemMap(index+1).samples = size(kinemPlan(6*(i)+1).sim.t,2);
                end
                feasKinemMap(index).Sw2St = kinemPlan(6*(i-1)+j).Sw2St;
                feasKinemMap(index).St2Sw = kinemPlan(6*(i-1)+j).St2Sw;
                % Compute IGM to find geometric feasibility
                out  = IGM( FBreal , j );
                % Further Logging
                feasKinemMap(index).leg = j;
                feasKinemMap(index).BiFootPos = FBreal;
                feasKinemMap(index).FootPos0 = F0;
                feasKinemMap(index).alpha = out.alpha;
                feasKinemMap(index).beta = out.beta;
                feasKinemMap(index).l = out.l;
                feasKinemMap(index).Fnl1 = out.feasFlags.Fnl1;
                feasKinemMap(index).Fnb1 = out.feasFlags.Fnb1;
                feasKinemMap(index).Fl1 = out.feasFlags.Fl1;
                feasKinemMap(index).Fl2 = out.feasFlags.Fl2;
                feasKinemMap(index).Fl3 = out.feasFlags.Fl3;
                feasKinemMap(index).Fa1 = out.feasFlags.Fa1;
                feasKinemMap(index).Fa2 = out.feasFlags.Fa2;
                feasKinemMap(index).Fa3 = out.feasFlags.Fa3;
                feasKinemMap(index).Fb1 = out.feasFlags.Fb1;
                feasKinemMap(index).Fb2 = out.feasFlags.Fb2;
                feasKinemMap(index).Fb3 = out.feasFlags.Fb3;
                feasKinemMap(index).COM = COM;
            end
            timeTemp = timeTemp + timeStamps(i);
        end 
    timeTemp2 = timeTemp2 + 6*timeStamps(i);    
    end
    feasKinemMap(1).feasValue = confirmGeomFeasibility( feasKinemMap );
    feasKinemMap(1).gait = kinemPlan(1).gait;
    feasKinemMap(1).Ts = kinemPlan(1).Ts;
  
end