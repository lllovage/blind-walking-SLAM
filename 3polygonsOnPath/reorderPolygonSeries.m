function polygonSeries = reorderPolygonSeries (polygonSeries)
    for i=1:size(polygonSeries,2)
        % Reorder stance feet numbers and coordinates
        x = polygonSeries(i).stCoords(1,:);
        y = polygonSeries(i).stCoords(2,:);
        [ord, ord] = sort([angle(complex(x-mean(x),y-mean(y)))]);
        index = 1:1:size(polygonSeries(i).stCoords,2);
        copyPolygonSeries = polygonSeries;
        polygonSeries(i).stFeet(ord) = copyPolygonSeries(i).stFeet(index);
        polygonSeries(i).stCoords(:,ord) = copyPolygonSeries(i).stCoords(:,index);
        
        % Reorder swing feet numbers and coordinates
        x = polygonSeries(i).swCoords(1,:);
        y = polygonSeries(i).swCoords(2,:);
        [ord, ord] = sort([angle(complex(x-mean(x),y-mean(y)))]);
        index = 1:1:size(polygonSeries(i).swCoords,2);
        copyPolygonSeries = polygonSeries;
        polygonSeries(i).swFeet(ord) = copyPolygonSeries(i).swFeet(index);
        polygonSeries(i).swCoords(:,ord) = copyPolygonSeries(i).swCoords(:,index);
    end
end