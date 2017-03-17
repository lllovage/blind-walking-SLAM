function [res, newParams] = solvePoly(params,t,mode)
    % mode->1 position, 2-> velocity, 3->acceleration
    if iscolumn(params)
    else params = params';
    end
    if mode == 1
        %------
        tMatrix = 1;
        for i=1:size(params,1)-1
            tMatrix = [tMatrix; t^i];
        end
        res = params'*tMatrix;
        newParams = params;
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
            newParams = [params(2:end);0];
            %------
        else
            res = 0;
            newParams = 0;
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
            newParams = [params(3:end);0];
            %------
        else
            res = 0;
            newParams = 0;
        end
    end
    
%     tMatrix = 1;
%         for i=1:size(params,1)-1
%             tMatrix = [tMatrix; t^i];
%         end
%     res = params'*tMatrix;
end