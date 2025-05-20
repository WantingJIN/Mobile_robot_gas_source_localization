% Plots robot trajectory with collected measurements, posterior PDF map and
% navigation planning results
clc; clear all

general_folder='/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/cluster_analysis';
experiment_path = uigetdir(general_folder);
experiment_file_path = [experiment_path,'/101'];
%experiment_file_path = general_folder;

anim_folder=[general_folder '/info'] ;
if(~exist(anim_folder,'dir'))
    mkdir(anim_folder);
end
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
    relative_measurements = dlmread([experiment_file_path, '/relative_measurements.csv']);
end
position_history = dlmread([experiment_file_path, '/position_history.csv']); %position_history
entropy = dlmread([experiment_file_path, '/entropy.csv']);
positions = dlmread([experiment_file_path, '/goal_position.csv']);
if clustering~=0
    cluster_bounding_boxes = dlmread([experiment_file_path, '/clustering_bounding_boxes.csv'])
end
real_source_position = params.source_position;

%if clustering~=0
% if (params.gas_feature == 0)
    % use gas concentration as gas feature
    %gas_information_map = dlmread([experiment_file_path,'/gas_information_map.csv']);
% end
% if (params.gas_feature == 1)
%     gas_information_map = dlmread([experiment_file_path,'/gas_hit_variance_map.csv']);
% end
%end

if (params.planning_strategy == 5)
    gas_information_map = dlmread([experiment_file_path,'/posterior_pdf_3D.csv']);
end 

x=linspace(3,12.75,20);
y=linspace(0,4,20);

trajectory = dlmread([experiment_file_path, '/trajectory.csv']);
trajectory = [positions(1,:); trajectory];

%% show measurement
f1 =figure('Position',[301.0000  100.4000  1500.6000  800.6000]);
subplot(1,3,1)
if (gas_feature == 1 )
    scatter(position_history(:,3),position_history(:,2),30,measurements(:,3),'MarkerFaceColor','flat','MarkerEdgeColor','k');
elseif (gas_feature == 0 || gas_feature == 2)
    scatter(position_history(:,3),position_history(:,2),30,measurements(:,2),'MarkerFaceColor','flat','MarkerEdgeColor','k');
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
    imagesc(y,x,reshape(pdf(i,:),params.nbins(1),params.nbins(2)));
%     % Etiqueta de número de celda en cada posición
%     nbins_X = params.nbins(1);
%     nbins_Y = params.nbins(2);
%     for row = 1:nbins_X
%         for col = 1:nbins_Y
%             cell_number = (col-1)*nbins_X + row;
%             % y(col) es la coordenada Y, x(row) es X (porque imagesc usa Y primero)
%             text(y(col), x(row), num2str(cell_number), 'Color', 'w', 'FontSize', 7, 'HorizontalAlignment', 'center');
%         end
%     end

    if map_nbr~=0
        patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
    end
    hold on
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    plot(positions(i,3),positions(i,2),'ko','MarkerSize',25,'MarkerFaceColor','m');
    hold on
    xlim([0 4])
    ylim([2 13])
    ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
    axis equal; axis([-0.1,4.1,2.4,12.6]);
    view(180,90);  colormap(mymap); 
    title(['Posterior pdf on X and Y \newline           iteration ', num2str(i)]);
    ax.TitleFontSizeMultiplier = 2;
    
    subplot(1,3,3)
    % -------plot gas measurements of each iter------------%
%     if (gas_feature == 1)
%         scatter(position_history(:,3),position_history(:,2),10,measurements(:,3),'MarkerFaceColor','flat','MarkerEdgeColor','k');
%     elseif (gas_feature == 0 || gas_feature == 2)
%         scatter(position_history(:,3),position_history(:,2),10,measurements(:,2),'MarkerFaceColor','flat','MarkerEdgeColor','k');
%     end

    % -------plot information map of each iter------------%
    % the last iteration the information map won't update
    if i == size(pdf,1)
         imagesc(y,x,reshape(gas_information_map(i-1,1:end),params.nbins(1), params.nbins(2))); hold on
         if params.planning_strategy == 5
             if clustering~=0
                cluster_num = cluster_bounding_boxes(i-1,1);
                if cluster_num > 0
                    for cluster_idx = 1 : cluster_num
                        x1 = cluster_bounding_boxes(i-1, 3 + (cluster_idx-1)*4);
                        y1 = cluster_bounding_boxes(i-1, 4 + (cluster_idx-1)*4);
                        x2 = cluster_bounding_boxes(i-1, 5 + (cluster_idx-1)*4);
                        y2 = cluster_bounding_boxes(i-1, 6 + (cluster_idx-1)*4);
                        rectangle('Position',[y1 x1 (y2-y1) (x2-x1)], 'EdgeColor',"#A2142F",'LineWidth',2)
                        hold on
                    end
                end
             end
         end
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
                imagesc(y,x,reshape(gas_information_map_this_iteration,params.nbins(1), params.nbins(2))); hold on % uncomment to plot gas
                if cluster_num > 0
                    for cluster_idx = 1 : cluster_num
                        disp(cluster_idx)
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
     % -------plot relativegas measurements of each iter------------%
    if (gas_feature == 2)
        scatter(position_history(:,3),position_history(:,2),30,relative_measurements(i,:),'MarkerFaceColor','flat','MarkerEdgeColor','k');
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
    plot(positions(i,3),positions(i,2),'ko','MarkerSize',25,'MarkerFaceColor','m');
    hold on
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
    imagesc(y,x,reshape(visited_list(1,1:end-1),params.nbins(1),params.nbins(2)));
    hold on
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    plot(trajectory(:,3),trajectory(:,2),'DisplayName','Trajectory','LineWidth',1,'LineStyle','--','Color',[1 0.600000023841858 0.7843137383461]);%lines
    if (gas_feature == 1)
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
    close all
end 

%% show dispersion map
% f =figure('Position',[601.0000  111.4000  697.6000  1450.6000]);
% for i=1:size(gas_information_map,1)
%     imagesc(y,x,reshape(gas_information_map(i,1:end-1),params.nbins(1), params.nbins(2)))
%     hold on
%     plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
%     hold on
%     patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
%     hold on
%     plot(positions(i,3),positions(i,2),'ko','MarkerSize',25,'MarkerFaceColor','m');
%     hold on
%     axis equal; axis([-0.1,4.1,2.4,12.6]);view(180,90);
%     title(['Concentration Variance Map \newline           iteration ', num2str(i)]);
%     xlabel('Y(m)'); ylabel('X(m)');
%     load MyColormap2; colormap(mymap); colorbar;caxis([0 Inf]);
%     hold off;
%     xlim([0 4])
%     ylim([2 13])
%     ax = gca; ax.XDir = 'reverse'; ax.YDir = 'reverse';
%     ax = gca;
%     ax.TitleFontSizeMultiplier = 3;
%     pause;
%     drawnow
% %     saveas(f, [anim_folder,'/',num2str(i),'.png']);
% end