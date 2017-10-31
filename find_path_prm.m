function [finalPath,prm,Costs] = find_path_prm (mapInflated,Waypoints,maxDist,density)



xrange = mapInflated.XWorldLimits(2) - mapInflated.XWorldLimits(1);
yrange = mapInflated.YWorldLimits(2) - mapInflated.YWorldLimits(1);

MapInflated = OccupancyGrid2Matrix(mapInflated); %Grid in matrix form
ratio = MapInflated.free/MapInflated.workspace;  %Ratio of free area
X_vec = [Waypoints.x(1,:); Waypoints.y(1,:)];
n_wp = size(X_vec,2);


tic
init_nodes = round(ratio*xrange*yrange*density);
prm = robotics.PRM;
prm.Map = mapInflated;
prm.NumNodes = init_nodes;
prm.ConnectionDistance = maxDist;%/mapInflated.Resolution;
update(prm);
toc

subplot(1,2,1); show(prm)


finalPath = [];
for i = 1:n_wp-1
    startLocation = X_vec(:,i)';
    endLocation = X_vec(:,i+1)';
    
    %Check if we know whether final position is occupied
    occval = getOccupancy(mapInflated,endLocation);
    if(occval)
        path = [];
        prm = [];
        display('There is a waypoint in an obstacle!');
    else

%         pause;
        tic;
        path = findpath(prm, startLocation, endLocation);
        toc;
        while isempty(path)
            % No feasible path found yet, increase the number of nodes
            prm.NumNodes = round(prm.NumNodes*1.1)

            if(prm.NumNodes > 1.5*init_nodes)
                path = [];
                break;
            end
            update(prm);

            % Search for a feasible path with the updated PRM
            path = findpath(prm, startLocation, endLocation);
            subplot(1,2,1); show(prm);

        end
    end
%     pause;
%     
    finalPath = [finalPath; path];
    Costs(i) = getPathCost(finalPath);
end