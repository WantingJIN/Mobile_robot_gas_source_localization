% Plots robot trajectory with collected measurements, posterior PDF map and
% navigation planning results
clc; clear all

general_folder='/home/wanting/Workspace/kheperaodor/plume_model_learning/Results/test_random_source_random_start_position/gt_wind/10';
% general_folder='/home/wanting/Workspace/kheperaodor/plume_model_learning/Results/ADE_vs_DDPM/ADE_FDM_gt_wind_relative/0';
experiment_path = uigetdir(general_folder);
experiment_file_path = [experiment_path,'/101'];
%experiment_file_path = general_folder;

params = my_text2struct([experiment_file_path,'/param_log.csv']);       
clustering = 1;
visited_listd_true = 0;
gas_feature = params.gas_feature;
pdf = dlmread([experiment_file_path, '/posterior_pdf_3D.csv']);
if isfile([experiment_file_path, '/visited_list.csv'])
     visited_list = dlmread([experiment_file_path, '/visited_list.csv']);
     visited_list_true = 1;
else
    disp('c')
end
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
if (gas_feature == 2)
    relative_measurements = dlmread([experiment_file_path, '/relative_history_meas.csv']);
end

position_history = dlmread([experiment_file_path, '/position_history.csv']); %position_history
entropy = dlmread([experiment_file_path, '/entropy.csv']);
positions = dlmread([experiment_file_path, '/goal_position.csv']);
if clustering~=0
    cluster_bounding_boxes = dlmread([experiment_file_path, '/clustering_bounding_boxes.csv']);
end
real_source_position = params.source_position;

if (params.planning_strategy == 5)
    gas_information_map = dlmread([experiment_file_path,'/posterior_pdf_3D.csv']);
end 

% read the range of the estimation parameter
estimation_range = params.min_maxs;
xmin = estimation_range(3);
xmax = estimation_range(4);
ymin = estimation_range(5);
ymax = estimation_range(6);


x=linspace(xmin,xmax,params.nbins(2));
y=linspace(ymin,ymax,params.nbins(3));

trajectory = dlmread([experiment_file_path, '/trajectory.csv']);
trajectory = [positions(1,:); trajectory];

%% show measurement
 f1 =figure('Position',[301.0000  100.4000  1500.6000  800.6000]);
subplot(1,3,1)
if (gas_feature == 0 )
    scatter(position_history(:,3),position_history(:,2),30,measurements(:,2),'MarkerFaceColor','flat','MarkerEdgeColor','k');
elseif (gas_feature == 2)
    scatter(position_history(:,3),position_history(:,2),30,measurements(:,3),'MarkerFaceColor','flat','MarkerEdgeColor','k');
end
load MyColormap2; colormap(mymap); 
axis equal; axis([-0.1,4.1,2.4,12.6]);
view(180,90);
colorbar;caxis([0 Inf]);
hold off;
xlim([0 4])
ylim([2 13])
ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
title(['        Trajectory \newline and Measurements']);
ax.TitleFontSizeMultiplier = 2;


%% show pdf
for i=1:size(pdf,1)
    subplot(1,3,2)
    imagesc(y,x,reshape(pdf(i,:),params.nbins(2),params.nbins(3)));
    if map_nbr~=0
        patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
    end
    hold on
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    plot(positions(i,3),positions(i,2),'ko','MarkerSize',10,'MarkerFaceColor','m');
    hold on
    % -------plot absolute/relative gas measurements of each iter------------%
    xlim([0 4])
    ylim([2 13])
    ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
    axis equal; axis([-0.1,4.1,2.4,12.6]);
    view(180,90);  
    load MyColormap2; colormap(mymap); 
    colorbar;caxis([0 Inf]);
    title(['Posterior pdf on X and Y \newline           iteration ', num2str(i)]);
    ax.TitleFontSizeMultiplier = 2;
    
    subplot(1,3,3)

    % -------plot information map of each iter------------%
    % the last iteration the information map won't update
    if i == size(pdf,1)
         imagesc(y,x,reshape(gas_information_map(i,1:end),params.nbins(1), params.nbins(2))); hold on
    else
        if params.planning_strategy == 5
            if clustering~=0
                cluster_num = cluster_bounding_boxes(i,1);
                ratio = cluster_bounding_boxes(i,2);
                gas_information_map_this_iteration = gas_information_map(i,1:end);
                max_gas = max(gas_information_map_this_iteration);
                min_gas = min(gas_information_map_this_iteration);
                threshold = ((max_gas-min_gas)*ratio +min_gas);
                gas_information_map_this_iteration(gas_information_map_this_iteration < threshold) = 0;
                imagesc(y,x,reshape(gas_information_map_this_iteration,params.nbins(2), params.nbins(3))); hold on % uncomment to plot gas
                if cluster_num > 0
                    for cluster_idx = 1 : cluster_num
                        disp(cluster_num)
                        x1 = cluster_bounding_boxes(i, 3 + (cluster_idx-1)*4);
                        y1 = cluster_bounding_boxes(i, 4 + (cluster_idx-1)*4);
                        x2 = cluster_bounding_boxes(i, 5 + (cluster_idx-1)*4);
                        y2 = cluster_bounding_boxes(i, 6 + (cluster_idx-1)*4);
                        rectangle('Position',[y1 x1 (y2-y1) (x2-x1)],'EdgeColor', "#FFB300",'LineWidth',2)
                        hold on
                    end
                end
            end
        end
    end

    % Extract the iteration numbers (last column)
    iteration_col = measurements(:, end);
    % Find indices of all measurements up to iteration k
    idx_until_current_idx = max(find(iteration_col <= i-1));
    
    if (gas_feature == 0)
        scatter(position_history(1:idx_until_current_idx,3),position_history(1:idx_until_current_idx,2),30, measurements(1:idx_until_current_idx,2),'MarkerFaceColor','flat','MarkerEdgeColor','k');
    end
    if (gas_feature == 2)
        scatter(position_history(1:idx_until_current_idx,3),position_history(1:idx_until_current_idx,2),30,relative_measurements(i,1:idx_until_current_idx),'MarkerFaceColor','flat','MarkerEdgeColor','k');
        hold on
    end
    plot(trajectory(:,3),trajectory(:,2),'DisplayName','Trajectory','LineWidth',1,'LineStyle','--','Color',[1 0.600000023841858 0.7843137383461]);%lines
    hold on
    %rectplot = @(x1,y1,x2,y2) rectangle('Position',[x1 y1 x2 y2]);
    
   
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    if map_nbr~=0
        patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
    end
    hold on
    plot(positions(i,3),positions(i,2),'ko','MarkerSize',10,'MarkerFaceColor','m');
    hold on
    % -------plot absolute/relative gas measurements of each iter------------%
    % Extract the iteration numbers (last column)
    iteration_col = measurements(:, end);
    % Find indices of all measurements up to iteration k
    idx_until_current_idx = max(find(iteration_col <= i-1));
    
    if (gas_feature == 0)
        scatter(position_history(1:idx_until_current_idx,3),position_history(1:idx_until_current_idx,2),30, measurements(1:idx_until_current_idx,2),'MarkerFaceColor','flat','MarkerEdgeColor','k');
    end
    if (gas_feature == 2)
        scatter(position_history(1:idx_until_current_idx,3),position_history(1:idx_until_current_idx,2),30,relative_measurements(i,1:idx_until_current_idx),'MarkerFaceColor','flat','MarkerEdgeColor','k');
        hold on
    end
    axis equal; axis([-0.1,4.1,2.4,12.6]);
    view(180,90);
%     if (params.info_feature == 0)
%         title(['Concentration variance map \newline           iteration ', num2str(i)]);
%     elseif (params.info_feature == 1)
     title(['Belief clustering result \newline           iteration ', num2str(i)]);
%     end
    xlabel('Y(m)'); ylabel('X(m)');     load MyColormap2; colormap(mymap); 
    colorbar;caxis([0 Inf]);
    hold off;
    xlim([0 4])
    ylim([2 13])
    ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
    ax.TitleFontSizeMultiplier = 2;
    pause
    drawnow
    % Save the figure
%     img_filename = [anim_folder, '/', num2str(i), '.png'];
%     try
%         saveas(f1, img_filename);
%         disp(['Image ', num2str(i), ' saved successfully.']);
%     catch
%         disp(['Error: Image ', num2str(i), ' not saved. Check the folder permissions or file path.']);
%     end


end

mat = reshape(pdf(i,:), params.nbins(1), params.nbins(2));
for row = 1:params.nbins(1)
    for col = 1:params.nbins(2)
        cell_number = (col-1)*params.nbins(1) + row;
        text(y(col), x(row), num2str(cell_number), 'Color', 'w', 'FontSize', 8);
    end
end


% Close the figure 
close(f1);
close all
%% show visited list
if visited_list_true~=0
    disp('show visited list') 
    f =figure('Position',[601.0000  111.4000  697.6000  1450.6000]);
    imagesc(y,x,reshape(visited_list(1,1:end-1),params.nbins(2),params.nbins(3)));
    hold on
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    plot(trajectory(:,3),trajectory(:,2),'DisplayName','Trajectory','LineWidth',1,'LineStyle','--','Color',[1 0.600000023841858 0.7843137383461]);%lines
    if (gas_feature == 2)
        scatter(position_history(:,3),position_history(:,2),10,measurements(:,3),'MarkerFaceColor','flat','MarkerEdgeColor','k');
    elseif (gas_feature == 0)
        scatter(position_history(:,3),position_history(:,2),10,measurements(:,2),'MarkerFaceColor','flat','MarkerEdgeColor','k');
    end
    load MyColormap2; colormap(mymap); 
    axis equal; axis([-0.1,4.1,2.4,12.6]);
    view(180,90);
%     colorbar;caxis([0 Inf]);
    hold off;
    xlim([0 4])
    ylim([2 13])
    ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
    camroll(90)
    title(['Trajectory and Visited Cells']);
    ax.TitleFontSizeMultiplier = 2;
    drawnow

    % Save the figure
%     img_filename = [anim_folder, '/visited.png'];
%     try
%         saveas(f, img_filename);
%         disp(['Image ', num2str(i), ' saved successfully.']);
%     catch
%         disp(['Error: Image ', num2str(i), ' not saved. Check the folder permissions or file path.']);
%     end
    % Close the figure 
    %close(f);
%     close all
end 
