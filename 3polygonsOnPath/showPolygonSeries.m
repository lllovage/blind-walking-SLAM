function showPolygonSeries(polygonSeries)
    for i=1:size(polygonSeries,2)
        % Stance feet drawing
        for j = 1:size(polygonSeries(i).stCoords,2)
            if j == size(polygonSeries(i).stCoords,2)
                k = 1;
            else
                k = j+1;
            end
                x1 = polygonSeries(i).stCoords(1,j);
                x2 = polygonSeries(i).stCoords(1,k);
                y1 = polygonSeries(i).stCoords(2,j);
                y2 = polygonSeries(i).stCoords(2,k);
                line([x1,x2],[y1,y2])
                plot(x1,y1,'gx')
                txt1 = ['\leftarrow ',num2str(polygonSeries(i).stFeet(j))];
                text(x1,y1,txt1)
        end
        
        %Swing feet drawing
        for j = 1:size(polygonSeries(i).swCoords,2)
%             if j == size(polygonSeries(i).swCoords,2)
%                 k = 1;
%             else
%                 k = j+1;
%             end
                 x1 = polygonSeries(i).swCoords(1,j);
%                 x2 = polygonSeries(i).swCoords(1,k);
                 y1 = polygonSeries(i).swCoords(2,j);
%                 y2 = polygonSeries(i).swCoords(2,k);
%                 line([x1,x2],[y1,y2])
                plot(x1,y1,'ko')
                txt1 = ['\leftarrow ',num2str(polygonSeries(i).swFeet(j))];
                text(x1,y1,txt1)
        end
        
        % ---------
    end
end