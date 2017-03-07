function res = solvePoly(params,t,mode)
    % mode->1 position, 2-> velocity, 3->acceleration
    if mode == 1
        %------
        tMatrix = 1;
        for i=1:size(params,1)-1
            tMatrix = [tMatrix; t^i];
        end
        res = params'*tMatrix;
        %------
    elseif mode == 2
        if size(params,1) > 1
            params(1) = 0;
            for i = 1:size(params,1)-1
                params(i+1) = i*params(i+1);
            end
            %------
            tMatrix = [0;1];
            for i=1:size(params,1)-2
                tMatrix = [tMatrix; t^i];
            end
            res = params'*tMatrix;
            %------
        else
            res = 0;
        end
    elseif mode == 3
        if size(params,1) > 2
            params(1) = 0;
            for i = 1:size(params,1)-1
                params(i+1) = i*params(i+1);
            end
            
            params(2) = 0;
            for i = 1:size(params,1)-2
                params(i+2) = i*params(i+2);
            end
            %------
            tMatrix = [1;1;1];
            for i=1:size(params,1)-3
                tMatrix = [tMatrix; t^i];
            end
            res = params'*tMatrix;
            %------
        else
            res = 0;
        end
    end
    
%     tMatrix = 1;
%         for i=1:size(params,1)-1
%             tMatrix = [tMatrix; t^i];
%         end
%     res = params'*tMatrix;
end