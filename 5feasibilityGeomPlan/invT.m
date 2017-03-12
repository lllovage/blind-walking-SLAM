function Tinv = invT(T)
    Tinv = [[T(1:3,1:3)'; zeros(1,3)],[-T(1:3,1:3)'*T(1:3,4);1]];
end