function Rres = compR (theta, dir)
    ct = cos(theta);
    st = sin(theta);
    if strcmp(dir,'x')
        Rres = [1,0,0; 0,ct,-st; 0,st,ct];
    elseif strcmp(dir,'y')
        Rres = [ct,0,st; 0,1,0; -st,0,ct];
    elseif strcmp(dir,'z')
        Rres = [ct,-st,0; st,ct,0; 0,0,1];
    else
        
    end
end