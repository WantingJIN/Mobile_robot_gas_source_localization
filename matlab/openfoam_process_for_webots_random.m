% Note: 
% 1. when the source height has changed in the setup -> Ux, Uy, CCx, CCy, when
% extact the wind at a certain level (0.1m) for instance, needs to be
% changed
% 2. when the obstacle height has changed in the setup -> threshold to
% select the obstacle region need to be adapted ->line 138 for instance

close all; clear all; clc;

% general folder
general_folder = '../openfoam/training_random/training_random_wind0.75_angle0/';
addpath("functions");
% universal grid
step = 0.1; % this should match the grid size in openfoam
x = round(0.2124:0.1586:15.9138, 4);
y = round(0.01:0.0632:3.9916, 4);
z = round(0:0.1:2, 4);
[X,Y,Z] = meshgrid(x,y,z);

% list of maps
folder_list = dir(general_folder);
folder_list = folder_list([folder_list.isdir]);
folder_list = folder_list(~ismember({folder_list.name},{'.','..','0'}));

%% for all maps i
for maps = 173
    folder = [general_folder num2str(maps)];
    
    % data
    [CC, U] = openfoam_steadyState_CC_U(folder);
    
    %% obstacles
    str = fileread([folder '/system/topoSetDict']);
    C = regexp(str,'boxes ([^\n\r]+);','match');
    C1 = textscan(C{1}(8:end-2),'(%f %f 0) (%f %f 1)','Delimiter',{')','(',' '});
    
    nbr_obstacles = size(C1{1},1);
    
    fileID = fopen([folder '/' num2str(maps) '_obstacles.txt'],'w');
    fprintf(fileID, '%d\r\n', nbr_obstacles);
    for i=1:nbr_obstacles
        fprintf(fileID, '%f %f %f %f\r\n', C1{1}(i), C1{2}(i), C1{3}(i), C1{4}(i));
    end
    fclose(fileID);
    
    
    %% flow def and SDF
   
    % universal grid
    Wx = round(2.75:0.1586:12.7418, 4);
    Wy = round(0.01:0.0632:3.9916, 4);
    [WX,WY] = meshgrid(Wx,Wy);
    
    % initialization
    flow_def = ones(length(WX), length(WY));
    sdf = ones(length(WX), length(WY)) * 9000;
    
    % obstacles
    for i=1:nbr_obstacles
        xmin = C1{1}(i); ymin = C1{2}(i); xmax = C1{3}(i); ymax = C1{4}(i);
        mxmin = closest_idx(Wx, xmin);
        mxmax = closest_idx(Wx, xmax);
        mymin = closest_idx(Wy, ymin);
        mymax = closest_idx(Wy, ymax);
        flow_def(mymin:mymax, mxmin:mxmax) = 0;
        flow_def(mymin:mymax, mxmin) = 5;
        flow_def(mymin:mymax, mxmax) = 5;
        flow_def(mymin, mxmin:mxmax) = 5;
        flow_def(mymax, mxmin:mxmax) = 5;
    end
    
    % input
    flow_def(WX==max(Wx)) = 3;
    % output
    flow_def(WX==min(Wx)) = 4;
    % walls
    flow_def(WY==min(Wy) | WY==max(Wy)) = 2;
    
    % remove borders between obstacle if they are stuck together    
    for i = find(flow_def==5)'
        if(flow_def(i-1)==0 && flow_def(i+1)==0)
            flow_def(i) = 0;
        end
        if(flow_def(i-64)==0 && flow_def(i+64)==0)
            flow_def(i) = 0;
        end
    end
    
    % find all obstacle borders
    [r,c] = find(flow_def == 5 | flow_def == 2);
    % for each cell
    for j=1:size(sdf,1)*size(sdf,2)
        for k=1:length(r)
            % we find the distance to each obstacle border cell
            dist = sqrt((WX(j)-WX(r(k),c(k)))^2 + (WY(j)-WY(r(k),c(k)))^2);
            % and we save the min
            if(dist< sdf(j))
                sdf(j) = dist;
            end
        end
    end
    sdf(flow_def == 0) = sdf(flow_def == 0) .* -1;
    
    % highlight obstacl area by giving it a big negative value
%     flow_def(flow_def == 5 | flow_def == 0 ) = -50;
    
    % save
    save([folder '/SDF.mat'], 'X', 'Y', 'sdf');
    save([folder '/flow_def.mat'], 'X', 'Y', 'flow_def');
    
    %% wind
    wind_x = zeros(length(Wx), length(Wy));
    wind_y = zeros(length(Wx), length(Wy));
    % load the txt file from openfoam
    % Load the Ux and Uy at the height of 0.1m
    Ux = U(ismember(CC(:,1),x) & CC(:,3)==0.1, 1);
    Uy = U(ismember(CC(:,1),x) & CC(:,3)==0.1, 2);
    CCx = CC(ismember(CC(:,1),x) & CC(:,3)==0.1, 1);
    CCy = CC(ismember(CC(:,1),x) & CC(:,3)==0.1, 2);
    for j=1:length(Ux)
        wind_x(Wy==CCy(j), Wx==CCx(j)) = Ux(j);
        wind_y(Wy==CCy(j), Wx==CCx(j)) = Uy(j);
    end
    % make sure wind=0 on obstacles
    wind_x(flow_def == -50 | flow_def == 2) = 0;
    wind_y(flow_def == -50 | flow_def == 2) = 0;
    % save
    save([folder '/wind_map.mat'], 'X', 'Y', 'wind_x', 'wind_y');
    
    %% obstacles
    obstacle_flag = ismembertol([X(:),Y(:),Z(:)], CC, 'ByRows', true);
    
    % 2D obstacle map
    X2 = X(:,:,1);
    Y2 = Y(:,:,1);
    Z2 = Z(:,:,1);
    % Note this number *20* depend on the height of the obstacle 
    % TODO: only extract obstacle at a fixed height level
    obstacle_map = sum(reshape(obstacle_flag.*2,length(y),length(x),length(z)),3) > 20;
    
    % the walls around the map
    obstacle_map(X2==step/2 | round(X2,4)==16-step/2 | Y2==step/2 | round(Y2,4)==4-step/2) = 0;
    
    % display
    imagesc(X2(:),Y2(:),obstacle_map);
    % colormap(gray);
    title(['map ' num2str(maps)])
    axis equal; axis tight;
    hold on;
    
    % save
    save([folder '/' num2str(maps) '_obstacle.mat'], 'X2', 'Y2', 'obstacle_map');
        
    %% stream
    wind_x = zeros(length(y), length(x), length(z));
    wind_y = zeros(length(y), length(x), length(z));
    wind_z = zeros(length(y), length(x), length(z));
    for j=1:length(U)
        wind_x(find(y==CC(j,2)), find(x==CC(j,1)),find(z==CC(j,3))) = U(j,1);
        wind_y(find(y==CC(j,2)), find(x==CC(j,1)),find(z==CC(j,3))) = U(j,2);
        wind_z(find(y==CC(j,2)), find(x==CC(j,1)),find(z==CC(j,3))) = U(j,3);
    end
    
    % plot
    [starty,startz] = meshgrid(0:0.5:4, 0.5:0.5:1.5);
    streamline(X,Y,Z, wind_x,wind_y,wind_z, 15.9*ones(size(starty)),starty,startz);
    % hold off;
    
    % save data
    d = [wind_x(:),wind_y(:),wind_z(:)]';
    fileID = fopen([folder '/' num2str(maps) '.txt'],'w');
    fprintf(fileID,'%d\r\n',length(d));
    fprintf(fileID,'%f %f %f\r\n',d);
    fclose(fileID);
        
    %% wind magnitude
    mag = sqrt(wind_x.^2 + wind_y.^2);
    mag_map = obstacle_map .* mag(:,:,10);
    imagesc(X2(:),Y2(:),mag_map);
    colorbar;
    hold off;
    
    %% save
    saveas(gcf,[folder '/' num2str(maps) '.png']);
end

%% helper function
function idx = closest_idx(array, value)
    [~,idx] = min(abs(array - value));
end
