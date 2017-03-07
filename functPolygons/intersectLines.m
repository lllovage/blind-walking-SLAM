function [x,y,noInt] = intersectLines(l1, l2)
epsilon = 0.001;
% Remember line layout l = [m,b,flag]
    if (l1(3) == 0 && l2(3) == 0) %Check flag
        if ((l1(1)<l2(1)-epsilon) || (l1(1)>l2(1)+epsilon)) % If lines have same slope THEN
            D = [-1, l1(1); -1, l2(1)];
            b = [-l1(2); -l2(2)];
            intersec = D\b;
            x = intersec(2);
            y = intersec(1);
            noInt = 0;
        else
            x = 0; y = 0; noInt = 1; % Lines parallel, noIntersection flag ON.
        end
    elseif (l1(3) == 1 && l2(3) == 0)
                x = l1(4);
                y = l2(1)*x+l2(2);
                noInt = 0;
    
    elseif (l1(3) == 0 && l2(3) == 1)
                    x = l2(4);
                    y = l1(1)*x+l1(2);
                    noInt = 0;
    elseif (l1(3) == 1 && l2(3) == 1)
                        x = 0; y = 0; noInt = 1; % Both slopes infinite
    end        
end