function [intPolygon, tooFar] = intersectPolygons ( polygons )
    %This function returns the intersection polygon between two polygons.
    %Notice that if no intersection is found (polygons separated) or only
    %one intersection is found (polygons touch each other on a vertex) then
    %a null intPolygon is returned and tooFar 
    %1. Extract lines from polygons
    maxNumFeet = 0;
    for i = 1:size(polygons,1)
        if maxNumFeet > size(polygons(i).coords,1)
        else
            maxNumFeet=size(polygons(i).coords,1);
        end      
    end
    lines = zeros(size( polygons,1 ),maxNumFeet,4);
    ranges = zeros(size( polygons,1 ),maxNumFeet,2,2);
    for i = 1:size( polygons,1 ) % Number of polygons
        stFeet = size(polygons(i).coords,1); % Number of vertices per polygon
        for j = 1:stFeet % Foot in consideration
            % if m=inf -> flag = 1; flag = 0 otherwise;
            if j~= stFeet
                fi = polygons(i).coords(j,:);
                fi1 = polygons(i).coords(j+1,:);
            else
                fi = polygons(i).coords(j,:);
                fi1 = polygons(i).coords(1,:);
            end
            
            [m,b,flag,xinf] = lineEquation(fi,fi1); % (Polygon, line number, [m,b,flag])
            % Notice ordering: lines(polygon, starting foot, [m, b, flag,xinf])
            % xinf-> If flag=1, xinf contains the value of x at which
            % m=inf. It is 0 otherwise.
            lines(i,j,1) = m;
            lines(i,j,2) = b;
            lines(i,j,3) = flag;
            lines(i,j,4) = xinf;
            %2 Extract ranges for each line
            % Notice ordering: ranges(polygon, starting foot, 1 or 2, [min,max])
            ranges(i,j,1,1:2) = [min(fi(1),fi1(1)),max(fi(1),fi1(1))]; % for x
            ranges(i,j,2,1:2) = [min(fi(2),fi1(2)),max(fi(2),fi1(2))]; % for y
        end
    end
    % 3. Mathematical intersection of lines of 2 different polygons
    % Notice ordering: intersections(lineP1,lineP2,[x,y,noInt])
    intersections = zeros(size(polygons(1).coords,1),size(polygons(2).coords,1),3);
    for i = 1:size(polygons(1).coords,1) %lines first polygon
        for j = 1:size(polygons(2).coords,1) % lines second polygon
            % Extract lines to intersect
            l1 = lines(1,i,:);
            l2 = lines(2,j,:);
            [x,y,noInt] = intersectLines(l1,l2); %if noInt = 1-> Lines parallel, 0 otherwise
            intersections(i,j,1) = x;
            intersections(i,j,2) = y;
            intersections(i,j,3) = noInt;
        end
    end
    
    %4. Filter out useful intersections
    % Notice ordering: ranges(polygon, starting foot, 1 or 2, [min,max])
                     % intersections(lineP1,lineP2,[x,y,noInt])
    coords = zeros(1000,1,2);
    count = 0;
    for i =  1:size(polygons(1).coords,1) % lines first polygon
        for j =  1:size(polygons(2).coords,1) % lines second polygon
            if (intersections(i,j,1) <= ranges(1,i,1,2)) && (ranges(1,i,1,1) <= intersections(i,j,1)) && ...
               (intersections(i,j,2) <= ranges(1,i,2,2)) && (ranges(1,i,2,1) <= intersections(i,j,2)) && ...
               (intersections(i,j,1) <= ranges(2,j,1,2)) && (ranges(2,j,1,1) <= intersections(i,j,1)) && ...
               (intersections(i,j,2) <= ranges(2,j,2,2)) && (ranges(2,j,2,1) <= intersections(i,j,2)) && ...
               (intersections(i,j,3) == 0)
               %coords = [coords; intersections(i,j,1:2)];
               count = count+1;
               coords(count,1,1:2) = intersections(i,j,1:2);
            end
        end        
    end
    coords = coords(1:count,:,:);
    %5. Add corners of existing polygons inside other polygons
    for i = 1:size(polygons,1)
        % Separate polygons from actual one to analyze insident corners
        if i ~= 1
            tempPolygons = [polygons(1:i-1,:); polygons(i+1:end,:)];
        else
            tempPolygons = polygons(i+1:end,:);
        end
        %Test for insident corners
        for j = 1:size(tempPolygons,1) % Iterate corners of temporal separated polygons on selected one
            testx = polygons(i).coords(:,1);
            testy = polygons(i).coords(:,2);
            polx = tempPolygons(j).coords(:,1);
            poly = tempPolygons(j).coords(:,2);
            % Test if points in [testx,testy] are inside of polygon [polx,poly]
            in = inpolygon(testx,testy,polx,poly);
            in = find(in);
            if size(in,1)>=1
                stack = size([testx(in),testy(in)],1);
                newcoords = reshape([testx(in),testy(in)],stack,1,2);
                coords = cat(1,coords,newcoords);
            end
        end
    end
     
    %6. Order the vertices and form the polygon structure
    % IF NO INTERSECTION: RETURN A VOID STRUCTURE AND RAISE FARAWAY FLAG
    if size(coords,1) == 0 || size(coords,1) == 1
        intPolygon = 0;
        tooFar = 1;
    else
        in = size(coords,1);
        x = reshape(coords(:,1,1),in,1);
        y = reshape(coords(:,1,2),in,1);
        %Remove repeated items
        coords = unique([x,y],'rows');
        x = coords(:,1);
        y = coords(:,2);
        [ord, ord] = sort([angle(complex(x-mean(x),y-mean(y)))]);
        coords=[x(ord),y(ord)];
        intPolygon.coords = coords;
        % whichever order as intersection polygon will never be used for
        % standing
        intPolygon.feet = ord;
        intPolygon.A = areaPolygon2D ( intPolygon );
        [intPolygon.centroid(1,1), intPolygon.centroid(2,1), intPolygon.centroid(3,1),intPolygon.centroid(4,1)] = centroidPolygon2D ( intPolygon );
        tooFar = 0;
    end
    
end