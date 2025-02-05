% Plots robot trajectory with collected measurements, posterior PDF map and
% navigation planning results
clc; clear all

general_folder='../Results/info_planning/MOX/map5/Dijkstra_Sum';
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
    relative_measurements = dlmread([experiment_file_path, '/relative_measurements.csv']);
end
position_history = dlmread([experiment_file_path, '/position_history.csv']); %position_history
entropy = dlmread([experiment_file_path, '/entropy.csv']);
positions = dlmread([experiment_file_path, '/goal_position.csv']);
real_source_position = params.source_position;

gas_score_map = dlmread([experiment_file_path,'/scores_map.csv']);
distance_map = dlmread([experiment_file_path,'/distance_map.csv']);
acc_info_map = dlmread([experiment_file_path,'/acc_info_map.csv']);

x=linspace(3,12.75,20);
y=linspace(0,4,20);

trajectory = dlmread([experiment_file_path, '/trajectory.csv']);
trajectory = [positions(1,:); trajectory];

%% show measurement
f1 =figure('Position',[301.0000  100.4000  1800.6000  800.6000]);
subplot(1,5,1)
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
xlabel('Y(m)'); ylabel('X(m)')
ax.TitleFontSizeMultiplier = 2;

%% show pdf
for i=1:size(pdf,1)
    subplot(1,5,2)
    imagesc(y,x,reshape(pdf(i,:),params.nbins(1),params.nbins(2)));
    if map_nbr~=0
        patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
    end
    hold on
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    plot(positions(i,3),positions(i,2),'ko','MarkerSize',25,'MarkerFaceColor','m');
    hold on
    axis equal; axis([-0.1,4.1,2.4,12.6]);
    xlim([0 4])
    ylim([2 13])
    ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
    view(180,90);  colormap(mymap); 
    title(['Posterior pdf  \newline iteration ', num2str(i)]);
    colorbar;caxis([0 Inf]);
    xlabel('Y(m)'); ylabel('X(m)')
    ax.TitleFontSizeMultiplier = 2;
    
    subplot(1,5,3)
    % -------plot disatance map of each iter------------%
    % the last iteration the distance map won't update
    if i == size(pdf,1)
         imagesc(y,x,reshape(distance_map(i-1,1:end),params.nbins(1), params.nbins(2))); hold on
    else
         imagesc(y,x,reshape(distance_map(i,1:end),params.nbins(1), params.nbins(2))); hold on % uncomment to plot gas        
    end
 

    plot(trajectory(:,3),trajectory(:,2),'DisplayName','Trajectory','LineWidth',1,'LineStyle','--','Color',[1 0.600000023841858 0.7843137383461]);%lines
    hold on
   
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    if map_nbr~=0
        patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
    end
    hold on
    plot(positions(i,3),positions(i,2),'ko','MarkerSize',25,'MarkerFaceColor','m');
    hold on
    axis equal; 
    view(180,90);
    title(['Distance map \newline iteration ', num2str(i)]);
    xlabel('Y(m)'); ylabel('X(m)');     load MyColormap2; colormap(mymap); 
    colorbar;caxis([0 Inf]);
    hold off;
    xlim([0 4])
    ylim([2 13])
    ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
    ax.TitleFontSizeMultiplier = 2;

    subplot(1,5,4)
    % -------plot acc info map of each iter------------%
    % the last iteration the acc info map won't update
    if i == size(pdf,1)
         imagesc(y,x,reshape(acc_info_map(i-1,1:end),params.nbins(1), params.nbins(2))); hold on
    else
         imagesc(y,x,reshape(acc_info_map(i,1:end),params.nbins(1), params.nbins(2))); hold on % uncomment to plot gas        
    end
 

    plot(trajectory(:,3),trajectory(:,2),'DisplayName','Trajectory','LineWidth',1,'LineStyle','--','Color',[1 0.600000023841858 0.7843137383461]);%lines
    hold on
   
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    if map_nbr~=0
        patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
    end
    hold on
    plot(positions(i,3),positions(i,2),'ko','MarkerSize',25,'MarkerFaceColor','m');
    hold on
    axis equal; 
    view(180,90);
    title(['Acc info map \newline iteration ', num2str(i)]);
    xlabel('Y(m)'); ylabel('X(m)');     load MyColormap2; colormap(mymap); 
    colorbar;caxis([0 Inf]);
    hold off;
    xlim([0 4])
    ylim([2 13])
    ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
    ax.TitleFontSizeMultiplier = 2;

    subplot(1,5,5)
    % -------plot score map of each iter------------%
    % the last iteration the score map won't update
    if i == size(pdf,1)
         imagesc(y,x,reshape(gas_score_map(i-1,1:end),params.nbins(1), params.nbins(2))); hold on
    else
         imagesc(y,x,reshape(gas_score_map(i,1:end),params.nbins(1), params.nbins(2))); hold on % uncomment to plot gas        
    end
  
    plot(trajectory(:,3),trajectory(:,2),'DisplayName','Trajectory','LineWidth',1,'LineStyle','--','Color',[1 0.600000023841858 0.7843137383461]);%lines
    hold on
   
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
    title(['Scoring function \newline iteration ', num2str(i)]);
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

% Close the figure 
close(f1);
close all

%% show visited list
% if visited_list_true~=0
%     disp('show visited list') 
%     f =figure('Position',[601.0000  111.4000  697.6000  1450.6000]);
%     imagesc(y,x,reshape(visited_list(1,1:end-1),params.nbins(1),params.nbins(2)));
%     hold on
%     plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
%     hold on
%     plot(trajectory(:,3),trajectory(:,2),'DisplayName','Trajectory','LineWidth',1,'LineStyle','--','Color',[1 0.600000023841858 0.7843137383461]);%lines
%     if (gas_feature == 1)
%         scatter(position_history(:,3),position_history(:,2),10,measurements(:,3),'MarkerFaceColor','flat','MarkerEdgeColor','k');
%     elseif (gas_feature == 0)
%         scatter(position_history(:,3),position_history(:,2),10,measurements(:,2),'MarkerFaceColor','flat','MarkerEdgeColor','k');
%     end
%     load MyColormap2; colormap(mymap); 
%     axis equal; axis([-0.1,4.1,2.4,12.6]);
%     view(180,90);
% %     colorbar;caxis([0 Inf]);
%     hold off;
%     xlim([0 4])
%     ylim([2 13])
%     ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
%     camroll(90)
%     title(['Trajectory and Visited Cells']);
%     ax.TitleFontSizeMultiplier = 2;
%     drawnow

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
    % close all
% end 

