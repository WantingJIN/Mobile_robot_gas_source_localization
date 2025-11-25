 % Plots robot trajectory with collected measurements, posterior PDF map and
% navigation planning results
clc; clear all

general_folder='/home/wanting/Workspace/kheperaodor/plume_model_learning/Results/online_wind_estimation/istropic_wind_background_average_relative/10';
% general_folder='/home/wanting/Workspace/kheperaodor/plume_model_learning/Results/ADE_vs_DDPM/ADE_FDM_gt_wind_relative/0';
experiment_path = uigetdir(general_folder);
experiment_file_path = [experiment_path,'/101'];

params = my_text2struct([experiment_file_path,'/param_log.csv']);       

pdf = dlmread([experiment_file_path, '/posterior_pdf_3D.csv']);

map_folder=[general_folder '/map'];
map_nbr = params.map_nbr;
if map_nbr~=0
    obst_file = ['../openfoam/test_maps/map' num2str(map_nbr) '/map' num2str(map_nbr) '_wind0.75/' num2str(map_nbr) '_obstacles.txt'];
    obst_fileID = fopen(obst_file);
    C = textscan(obst_fileID,'%f %f %f %f', 'HeaderLines', 1);
    for k=1:length(C{1,1})
                obstacles_x(:,k) = [C{1,1}(k) , C{1,3}(k), C{1,3}(k) , C{1,1}(k)];
                obstacles_y(:,k) = [C{1,2}(k) , C{1,2}(k), C{1,4}(k) , C{1,4}(k)];
    end
end

measurements = dlmread([experiment_file_path, '/measurement_history.csv']); %measurement_historyobstacles_x

position_history = dlmread([experiment_file_path, '/position_history.csv']); %position_history
entropy = dlmread([experiment_file_path, '/entropy.csv']);
positions = dlmread([experiment_file_path, '/goal_position.csv']);

real_source_position = params.source_position;

% read the range of the estimation parameter
xmin = 3.0;
xmax = 12.75;
ymin = 0.0;
ymax = 4.0;

Nx = 100;
Ny = 40;


x=linspace(xmin,xmax,Nx);
y=linspace(ymin,ymax,Ny);
[X, Y] = meshgrid(y, x);   % note: meshgrid swaps (row,col)
 
% Load data
u = dlmread([experiment_file_path, '/wind_field_u.csv']);   % size: Nx × Ny
v = dlmread([experiment_file_path, '/wind_field_v.csv']);   % size: Nx × Ny


trajectory = dlmread([experiment_file_path, '/trajectory.csv']);
trajectory = [positions(1,:); trajectory];

 f1 =figure('Position',[301.0000  100.4000  1500.6000  800.6000]);

 %% show pdf
for i=1:size(u,1)
    U = reshape(u(i,:) , Nx , Ny);
    V = reshape(v(i,:) , Nx , Ny);
    
    %% --- Compute magnitude of velocity ---
    Speed = sqrt(U.^2 + V.^2);

    %% --- Plot base color map (wind speed magnitude) ---
%     figure('Color', 'w');
    imagesc(y, x, Speed);     % color map for speed
    set(gca, 'YDir', 'normal'); % ensure Y increases upward
    axis equal tight;
    colormap(turbo);          % modern colormap
    colorbar;
    title('Wind Field Magnitude and Direction', 'FontWeight', 'bold');
    xlabel('x'); ylabel('y');

    %% --- Overlay quiver plot (wind direction) ---
    hold on;
    % Optionally downsample vectors for clarity
%     ds = max(floor(Nx/20),1);  % reduce density if grid is large
    ds = 3;
    quiver(X(1:ds:end, 1:ds:end), ...
           Y(1:ds:end, 1:ds:end), ...
           V(1:ds:end, 1:ds:end), ...
           U(1:ds:end, 1:ds:end), ...
           'k', 'LineWidth', 1.2, 'AutoScaleFactor', 1);
    hold on;

   
    if map_nbr~=0
        patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
    end
    hold on
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    axis equal;
    xlabel('x'); ylabel('y');
    title('Wind field (u,v)');
    hold on
    
    plot(positions(i,3),positions(i,2),'ko','MarkerSize',10,'MarkerFaceColor','m');
    hold on
% 
%     % Extract the iteration numbers (last column)
%     iteration_col = measurements(:, end);
%     % Find indices of all measurements up to iteration k
%     idx_until_current_idx = max(find(iteration_col <= i-1));
%     
%     if (gas_feature == 0)
%         scatter(position_history(1:idx_until_current_idx,3),position_history(1:idx_until_current_idx,2),30, measurements(1:idx_until_current_idx,2),'MarkerFaceColor','flat','MarkerEdgeColor','k');
%     end
%     if (gas_feature == 2)
%         scatter(position_history(1:idx_until_current_idx,3),position_history(1:idx_until_current_idx,2),30,relative_measurements(i,1:idx_until_current_idx),'MarkerFaceColor','flat','MarkerEdgeColor','k');
%         hold on
%     end
    plot(trajectory(:,3),trajectory(:,2),'DisplayName','Trajectory','LineWidth',1,'LineStyle','--','Color',[1 0.600000023841858 0.7843137383461]);%lines
    hold off
%     %rectplot = @(x1,y1,x2,y2) rectangle('Position',[x1 y1 x2 y2]);
%     
%    
%     plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
%     hold on
%     if map_nbr~=0
%         patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
%     end
%     hold on
%     plot(positions(i,3),positions(i,2),'ko','MarkerSize',10,'MarkerFaceColor','m');
%     hold on
%     % -------plot absolute/relative gas measurements of each iter------------%
%     % Extract the iteration numbers (last column)
%     iteration_col = measurements(:, end);
%     % Find indices of all measurements up to iteration k
%     idx_until_current_idx = max(find(iteration_col <= i-1));
%     
%     if (gas_feature == 0)
%         scatter(position_history(1:idx_until_current_idx,3),position_history(1:idx_until_current_idx,2),30, measurements(1:idx_until_current_idx,2),'MarkerFaceColor','flat','MarkerEdgeColor','k');
%     end
%     if (gas_feature == 2)
%         scatter(position_history(1:idx_until_current_idx,3),position_history(1:idx_until_current_idx,2),30,relative_measurements(i,1:idx_until_current_idx),'MarkerFaceColor','flat','MarkerEdgeColor','k');
%         hold on
%     end
%     axis equal; axis([-0.1,4.1,2.4,12.6]);
%     view(180,90);
%     title(['Belief clustering result \newline           iteration ', num2str(i)]);
% %     end
%     xlabel('Y(m)'); ylabel('X(m)');     load MyColormap2; colormap(mymap); 
%     colorbar;caxis([0 Inf]);
%     hold off;
%     xlim([0 4])
%     ylim([2 13])
%     ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
%     ax.TitleFontSizeMultiplier = 2;
    pause
    drawnow


end

% Close the figure 
close(f1);
close all

