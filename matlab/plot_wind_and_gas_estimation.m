% Plots robot trajectory with collected measurements, posterior PDF map and
% navigation planning results
clc; clear all

general_folder='/home/wanting/Workspace/kheperaodor/plume_model_learning/Results/physics_encoded_GSL/test_new_belief_clustering/dynamic_sigma_cdf/10';
% general_folder='/home/wanting/Workspace/kheperaodor/plume_model_learning/Results/ADE_vs_DDPM/ADE_FDM_gt_wind_relative/0';
experiment_path = uigetdir(general_folder);
experiment_file_path = [experiment_path,'/101'];

params = my_text2struct([experiment_file_path,'/param_log.csv']);       

pdf = dlmread([experiment_file_path, '/posterior_pdf_3D.csv']);

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

%% extract wind field estimation of this iteration
subplot(1,3,1)
iter = size(pdf,1)-1;

U = reshape(u(iter,:) , Nx , Ny);
V = reshape(v(iter,:) , Nx , Ny);

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
ds = 1;
quiver(X(1:ds:end, 1:ds:end), ...
       Y(1:ds:end, 1:ds:end), ...
       V(1:ds:end, 1:ds:end), ...
       U(1:ds:end, 1:ds:end), ...
       'k', 'LineWidth', 1, 'AutoScaleFactor', 0.5);
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

plot(positions(iter,3),positions(iter,2),'ko','MarkerSize',10,'MarkerFaceColor','m');
hold on

plot(trajectory(:,3),trajectory(:,2),'DisplayName','Trajectory','LineWidth',1,'LineStyle','--','Color',[1 0.600000023841858 0.7843137383461]);%lines
hold off


gas_maps_this_iteration = dlmread([experiment_file_path,'/gas_concentration_map_',num2str(iter),'.csv']);
gas_maps_ground_truth = dlmread([experiment_file_path,'/gas_concentration_map.csv']);
x=linspace(2.75,12.75,20);
y=linspace(0,4,20);

for i = 1:size(gas_maps_ground_truth, 1) %400
    subplot(1,3,2)
    source_position = gas_maps_ground_truth(i, 1:2);
 
    % be careful: the reshape function will first fill the column and then
    % row: Use the revert matrix!!
    imagesc(y, x, reshape(gas_maps_ground_truth(i, 3:end), 20, 20)');
    % imagesc(y, x, reshape(pdf(i,:), 20, 20))
    
    hold on
    % plot(positions(i,3), positions(i,2), 'ko', 'MarkerSize', 25, 'MarkerFaceColor', 'm');
    plot(source_position(2), source_position(1), 'p', 'MarkerSize', 40, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', [51/255 153/255 1]);
    hold on
    if map_nbr~=0
        patch(obstacles_y, obstacles_x, [0 0 0], 'DisplayName', 'Obstacle', 'LineStyle', 'none');
    end
    axis equal; axis([-0.1, 4.1, 2.4, 12.6]); view(180,90);
    title(['Ground Truth Gas Map \newline Source ', num2str(i)]);
    xlabel('Y(m)'); ylabel('X(m)');
    load MyColormap2; colormap(mymap); colorbar; caxis([0 Inf]);
    % annotation(f, 'textbox', [0.254287457153379 0.025796129034299 0.104365076491284 0.0473684201637904], 'String', {'Measurement:', num2str(measurements(i,2))}, 'LineStyle', 'none', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 24);
    % annotation(f, 'textbox', [0.60287457153379 0.025796129034299 0.104365076491284 0.0473684201637904], 'String', {'Entropy:', num2str(entropy(i,1))}, 'LineStyle', 'none', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 24);
    hold off;
    xlim([0 4])
    ylim([2 13])
    ax = gca; ax.XDir = 'reverse'; ax.YDir = 'reverse';
    ax = gca;
    ax.TitleFontSizeMultiplier = 1;
    
    subplot(1,3,3)
    source_position = gas_maps_this_iteration(i, 1:2);
 
    % be careful: the reshape function will first fill the column and then
    % row: Use the revert matrix!!
    imagesc(y, x, reshape(gas_maps_this_iteration(i, 3:end), 20, 20)');
    % imagesc(y, x, reshape(pdf(i,:), 20, 20))
    
    hold on
    % plot(positions(i,3), positions(i,2), 'ko', 'MarkerSize', 25, 'MarkerFaceColor', 'm');
    plot(source_position(2), source_position(1), 'p', 'MarkerSize', 40, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', [51/255 153/255 1]);
    hold on
    if map_nbr~=0
        patch(obstacles_y, obstacles_x, [0 0 0], 'DisplayName', 'Obstacle', 'LineStyle', 'none');
    end
    axis equal; axis([-0.1, 4.1, 2.4, 12.6]); view(180,90);
    title(['Current Estimated Gas Map \newline Source ', num2str(i)]);
    xlabel('Y(m)'); ylabel('X(m)');
    load MyColormap2; colormap(mymap); colorbar; caxis([0 Inf]);
    % annotation(f, 'textbox', [0.254287457153379 0.025796129034299 0.104365076491284 0.0473684201637904], 'String', {'Measurement:', num2str(measurements(i,2))}, 'LineStyle', 'none', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 24);
    % annotation(f, 'textbox', [0.60287457153379 0.025796129034299 0.104365076491284 0.0473684201637904], 'String', {'Entropy:', num2str(entropy(i,1))}, 'LineStyle', 'none', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 24);
    hold off;
    xlim([0 4])
    ylim([2 13])
    ax = gca; ax.XDir = 'reverse'; ax.YDir = 'reverse';
    ax = gca;
    ax.TitleFontSizeMultiplier = 1;
    
    pause;
    drawnow;
    

end
close all



% Close the figure 
% close(f1);
% close all
