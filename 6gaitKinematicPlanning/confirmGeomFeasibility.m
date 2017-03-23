function feasVal = confirmGeomFeasibility( feasGeomMap )
    % The lower feasVal, the "more feasible" the trajectory. Naturally it
    % must be zero for accepting the geometric path into kinematic planning phase.
     feasVal = sum(sum([[feasGeomMap(:).Fnl1]', [feasGeomMap(:).Fnb1]',...
                   [feasGeomMap(:).Fl1]', [feasGeomMap(:).Fl2]', ...
                   [feasGeomMap(:).Fl3]', [feasGeomMap(:).Fa1]', ...
                   [feasGeomMap(:).Fa2]', [feasGeomMap(:).Fa3]']));
    if feasVal ~=0
       warning(['confirmGeomFeasibility: Complete feasibility value different to 0. geomFeasVal = ' num2str(feasVal)]); 
    end
end