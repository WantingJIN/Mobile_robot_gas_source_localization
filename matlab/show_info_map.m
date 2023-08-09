clear all; close all; clc;

%general_folder='/home/wjin/Workspace/continuous_STE/Results/results_test_1_sense_reverse_model/1/';
general_folder='/home/wjin/Workspace/continuous_STE/Results/0809_pdf_clustering_ratio_10/2';
experiment_path = uigetdir(general_folder);
experiment_file_path = [experiment_path,'/101'];
%experiment_file_path = general_folder;

%general_folder='/home/wjin/Workspace/continuous_STE/Results/Experimental_Results/Results_0223/2023-02-23_15-23/pc';

pdf = dlmread([experiment_file_path, '/posterior_pdf_3D.csv']);
%pdf = dlmread([experiment_file_path,'/posterior_pdf_3D.csv']);
map_folder=[general_folder '/map'];
params = my_text2struct([experiment_file_path,'/param_log.csv']); 
map_nbr = params.map_nbr;
obst_file = ['/home/wjin/Workspace/continuous_STE/Khepera/Data/Obstacle_lists/' num2str(map_nbr) '_obstacles.txt'];
obst_fileID = fopen(obst_file);
C = textscan(obst_fileID,'%f %f %f %f', 'HeaderLines', 1);
for k=1:length(C{1,1})
            obstacles_x(:,k) = [C{1,1}(k) , C{1,3}(k), C{1,3}(k) , C{1,1}(k)];
            obstacles_y(:,k) = [C{1,2}(k) , C{1,2}(k), C{1,4}(k) , C{1,4}(k)];
end 
measurements = dlmread([experiment_file_path, '/measurement_history.csv']); %measurement_historyobstacles_x
position_history = dlmread([experiment_file_path, '/position_history.csv']); %position_history
entropy = dlmread([experiment_file_path, '/entropy.csv']);
positions = dlmread([experiment_file_path, '/goal_position.csv']);
cluster_bounding_boxes = dlmread([experiment_file_path, '/clustering_bounding_boxes.csv'])
real_source_position = params.source_position;
if (params.gas_feature == 0)
    % use gas concentration as gas feature
    gas_information_map = dlmread([experiment_file_path,'/gas_information_map.csv']);
end
if (params.gas_feature == 1)
    gas_information_map = dlmread([experiment_file_path,'/gas_hit_variance_map.csv']);
end

x=linspace(3,12.75,20);
y=linspace(0,4,20);

trajectory = dlmread([experiment_file_path, '/trajectory.csv']);
trajectory = [positions(1,:); trajectory];
gas_feature = params.gas_feature;
%% show measurement
f1 =figure('Position',[301.0000  100.4000  1500.6000  800.6000]);
subplot(1,3,1)
if (gas_feature == 1)
    scatter(position_history(:,3),position_history(:,2),10,measurements(:,3),'MarkerFaceColor','flat','MarkerEdgeColor','k');
elseif (gas_feature == 0)
    scatter(position_history(:,3),position_history(:,2),10,measurements(:,2),'MarkerFaceColor','flat','MarkerEdgeColor','k');
end
load MyColormap2; colormap(mymap); 
axis equal; axis([-0.1,4.1,2.4,12.6]);
view(180,90);
colorbar;caxis([0 Inf]);
hold off;
xlim([0 4])
ylim([2 13])
ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
title(['Trajectory and Measurements']);
ax.TitleFontSizeMultiplier = 2;
%% show pdf
for i=1:size(pdf,1)
    subplot(1,3,2)
    imagesc(y,x,reshape(pdf(i,:),params.nbins(1),params.nbins(2)));
    patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
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
    % the last iteration the gas variance map won't update
    if i == size(pdf,1)
        imagesc(y,x,reshape(gas_information_map(i-1,1:end-1),params.nbins(1), params.nbins(2))); hold on
        if params.planning_strategy == 5
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
    else
    if params.planning_strategy == 5
        cluster_num = cluster_bounding_boxes(i,1);
        ratio = cluster_bounding_boxes(i,2);
        gas_information_map_this_iteration = gas_information_map(i,1:end-1);
        max_gas = max(gas_information_map_this_iteration);
        min_gas = min(gas_information_map_this_iteration);
        threshold = ((max_gas-min_gas)*ratio +min_gas);
        gas_information_map_this_iteration(gas_information_map_this_iteration < threshold) = 0;
        imagesc(y,x,reshape(gas_information_map_this_iteration,params.nbins(1), params.nbins(2))); hold on
        if cluster_num > 0
            for cluster_idx = 1 : cluster_num
                x1 = cluster_bounding_boxes(i, 3 + (cluster_idx-1)*4);
                y1 = cluster_bounding_boxes(i, 4 + (cluster_idx-1)*4);
                x2 = cluster_bounding_boxes(i, 5 + (cluster_idx-1)*4);
                y2 = cluster_bounding_boxes(i, 6 + (cluster_idx-1)*4);
                rectangle('Position',[y1 x1 (y2-y1) (x2-x1)],'EdgeColor', "#A2142F",'LineWidth',2)
                hold on
            end
        end
    end
end
    plot(trajectory(:,3),trajectory(:,2),'DisplayName','Trajectory','LineWidth',1,'LineStyle','--','Color',[1 0.600000023841858 0.7843137383461]);%lines
    hold on
    %rectplot = @(x1,y1,x2,y2) rectangle('Position',[x1 y1 x2 y2]);
    
    
%     if (gas_feature == 1)
%         scatter(position_history(:,3),position_history(:,2),10,measurements(:,3),'MarkerFaceColor','flat','MarkerEdgeColor','k');
%     elseif (gas_feature == 0)
%         scatter(position_history(:,3),position_history(:,2),10,measurements(:,2),'MarkerFaceColor','flat','MarkerEdgeColor','k');
%     end
    hold on
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
    hold on
    plot(positions(i,3),positions(i,2),'ko','MarkerSize',25,'MarkerFaceColor','m');
    hold on
    axis equal; axis([-0.1,4.1,2.4,12.6]);
    view(180,90);
    if (params.info_feature == 0)
        title(['Concentration variance map \newline           iteration ', num2str(i)]);
    elseif (params.info_feature == 1)
        title(['Concentration mean map \newline           iteration ', num2str(i)]);
    end
    xlabel('Y(m)'); ylabel('X(m)');     load MyColormap2; colormap(mymap); 
    colorbar;caxis([0 Inf]);
    hold off;
    xlim([0 4])
    ylim([2 13])
    ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
    ax.TitleFontSizeMultiplier = 2;
    pause;
    drawnow
    %     saveas(f, [anim_folder,'/',num2str(i),'.png']);
end
close all
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