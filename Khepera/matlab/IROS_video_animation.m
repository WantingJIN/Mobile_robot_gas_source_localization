clear all; close all; clc;
% Work station
work_station = "Desktop";
%work_station = "Laptop"
if work_station == "Desktop"
    root_path = '/home/wjin/Workspace/continuous_STE/Khepera';
else
    root_path = '/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera';
end
anim_folder = [root_path, '/IROS_2025_Results/ani']   
general_folder=[root_path, '/IROS_2025_Results/Adp_Dijkstra/19-48'];

experiment_path = [general_folder,]; 
folder_name_k4 = [experiment_path, '/k4'];
folder_name_pc = [experiment_path, '/pc'];  

pdf = dlmread([folder_name_pc, '/posterior_pdf_3D.csv']);

params = my_text2struct([folder_name_pc,'/param_log.csv']); 
map_nbr = params.map_nbr;
obst_file = [root_path '/Data/Obstacle_lists/' num2str(map_nbr) '_obstacles.txt'];
obst_fileID = fopen(obst_file);
C = textscan(obst_fileID,'%f %f %f %f', 'HeaderLines', 1);
for k=1:length(C{1,1})
            obstacles_x(:,k) = [C{1,1}(k) , C{1,3}(k), C{1,3}(k) , C{1,1}(k)];
            obstacles_y(:,k) = [C{1,2}(k) , C{1,2}(k), C{1,4}(k) , C{1,4}(k)];
end 
measurements = dlmread([folder_name_pc, '/measurement_history.csv']); %measurement_historyobstacles_x
position_history = dlmread([folder_name_pc, '/position_history.csv']); %position_history
positions = dlmread([folder_name_k4, '/goal_position.csv']);
entropy = dlmread([folder_name_pc, '/entropy.csv']);
source_postion_file_name = [folder_name_pc '/source_position.txt'];
source_file = fopen(source_postion_file_name, 'r');
real_source_position = fscanf(source_file,'%f %f');
real_source_position(3) = 0.1;

gas_score_map = dlmread([folder_name_pc,'/scores_map.csv']);
distance_map = dlmread([folder_name_pc,'/distance_map.csv']);
acc_info_map = dlmread([folder_name_pc,'/acc_info_map.csv']);

x=linspace(3,12.75,20);
y=linspace(0,4,20);

trajectory = dlmread([folder_name_k4, '/trajectory.csv']);
trajectory = [positions(1,:); trajectory];

if (params.info_feature == 2)
    gas_information_map = dlmread([folder_name_pc,'/posterior_pdf_3D.csv']);
end 
x=linspace(3,12.75,20);
y=linspace(0,4,20);


% %% show measurement
% f1 =figure('Position',[301.0000  100.4000  470.6000  800.6000]);
% [ha, pos] = tight_subplot(2,2,[.09 .1],[.018 .08],[.03 .01])
% subplot(1,5,1)
% scatter(position_history(:,3),position_history(:,2),30,measurements(:,3),'MarkerFaceColor','flat','MarkerEdgeColor','k');
% load MyColormap2; colormap(mymap); 
% axis equal; axis([-0.1,4.1,2.4,12.6]);
% view(180,90);
% colorbar;caxis([0 Inf]);
% hold off;
% xlim([0 4])
% ylim([2 13])
% ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
% title(['        Trajectory \newline and Measurements']);
% xlabel('Y(m)'); ylabel('X(m)')
% ax.TitleFontSizeMultiplier = 2;

%% show pdf
% for i=1:size(pdf,1)
%     axes(ha(1))
%     imagesc(y,x,reshape(pdf(i,:),params.nbins(1),params.nbins(2)));hold on
%     if map_nbr~=0
%         patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
%     end
%     hold on
%     plot(real_source_position(2),real_source_position(1),'p','MarkerSize',20,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
%     hold on
%     plot(positions(i,3),positions(i,2),'ko','MarkerSize',12,'MarkerFaceColor','m');
%     hold on
% 
%     axis equal; 
%     view(180,90);
%     title(['     PDF  \newline iteration ', num2str(i)]);
%     xlabel('Y(m)'); ylabel('X(m)');     load MyColormap2; colormap(mymap); 
%     colorbar;%caxis([0 Inf]);
%     hold off;
%     xlim([0 4])
%     ylim([2 13])
%     ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
%     ax.TitleFontSizeMultiplier = 1.5;
% 
%     axes(ha(2))
%     % -------plot disatance map of each iter------------%
%     % the last iteration the distance map won't update
%     if i == size(pdf,1)
%          imagesc(y,x,reshape(distance_map(i-1,1:end),params.nbins(1), params.nbins(2))); hold on
%     else
%          imagesc(y,x,reshape(distance_map(i,1:end),params.nbins(1), params.nbins(2))); hold on % uncomment to plot gas        
%     end
% 
% 
%     plot(real_source_position(2),real_source_position(1),'p','MarkerSize',20,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
%     hold on
%     if map_nbr~=0
%         patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
%     end
%     hold on
%     plot(positions(i,3),positions(i,2),'ko','MarkerSize',12,'MarkerFaceColor','m');
%     hold on
%     axis equal; 
%     view(180,90);
%     title(['Distance Map']);
%     xlabel('Y(m)'); ylabel('X(m)');     load MyColormap2; colormap(mymap); 
%     colorbar;%caxis([0 Inf]);
%     hold off;
%     xlim([0 4])
%     ylim([2 13])
%     ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
%     ax.TitleFontSizeMultiplier = 1.5;
% 
%     axes(ha(3))
%     % -------plot acc info map of each iter------------%
%     % the last iteration the acc info map won't update
%     if i == size(pdf,1)
%          imagesc(y,x,reshape(acc_info_map(i-1,1:end),params.nbins(1), params.nbins(2))); hold on
%     else
%          imagesc(y,x,reshape(acc_info_map(i,1:end),params.nbins(1), params.nbins(2))); hold on % uncomment to plot gas        
%     end
% 
%     plot(real_source_position(2),real_source_position(1),'p','MarkerSize',20,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
%     hold on
%     if map_nbr~=0
%         patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
%     end
%     hold on
%     plot(positions(i,3),positions(i,2),'ko','MarkerSize',12,'MarkerFaceColor','m');
%     hold on
%     axis equal; 
%     view(180,90);
%     title(['Cumulative \newline Info Map']);
%     xlabel('Y(m)'); ylabel('X(m)');     load MyColormap2; colormap(mymap); 
%     colorbar;%caxis([0 Inf]);
%     hold off;
%     xlim([0 4])
%     ylim([2 13])
%     ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
%     ax.TitleFontSizeMultiplier = 1.5;
% 
%     axes(ha(4))
%     % -------plot score map of each iter------------%
%     % the last iteration the score map won't update
%     current_gas_score = [];
%     if i == size(pdf,1)
%          current_gas_score = gas_score_map(i-1,1:end);
%     else
%          current_gas_score = gas_score_map(i,1:end);
%     end
%     current_gas_score(current_gas_score ~= 0) = current_gas_score(current_gas_score ~= 0) + 1.0;
%     imagesc(y,x,reshape(current_gas_score,params.nbins(1), params.nbins(2))); hold on
%     [max_info,max_info_iter] = max(current_gas_score);
%     max_info_y_iter = floor(max_info_iter/params.nbins(2));
%     max_info_x_iter = max_info_iter - max_info_y_iter * params.nbins(1);
%     if max_info_x_iter ==0 
%         max_info_x_iter = params.nbins(1);
%     end
%     rectangle('Position',[y(max_info_y_iter+1)-0.1 x(max_info_x_iter)-0.25 0.2 0.5], "EdgeColor","#77AC30", "LineWidth",2);hold on
% 
%     % to improve the visualization of the video, we added some value to all
%     % the element that are not equal to zero to make it positive
% 
%     plot(real_source_position(2),real_source_position(1),'p','MarkerSize',20,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
%     hold on
%     if map_nbr~=0
%         patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
%     end
%     hold on
%     plot(positions(i,3),positions(i,2),'ko','MarkerSize',12,'MarkerFaceColor','m');
%     hold on
%     axis equal; axis([-0.1,4.1,2.4,12.6]);
%     view(180,90);
%     title(['Score Map']);
%     xlabel('Y(m)'); ylabel('X(m)');     load MyColormap2; colormap(mymap); 
%     colorbar;%caxis([0 Inf]);
%     hold off;
%     xlim([0 4])
%     ylim([2 13])
%     ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
%     ax.TitleFontSizeMultiplier = 1.5;
% 
% 
%     pause
%     drawnow
% 
%     %Save the figure
%     % img_filename = [anim_folder, '/', num2str(i), '.png'];
%     % try
%     %     saveas(f1, img_filename);
%     %     disp(['Image ', num2str(i), ' saved successfully.']);
%     % catch
%     %     disp(['Error: Image ', num2str(i), ' not saved. Check the folder permissions or file path.']);
%     % end
% 
% 
% end

% plot for iteration 0
pdf_initial = zeros(1, 400);
pdf_initial(:) = 1/400;
f2 = figure();
imagesc(y,x,reshape(pdf_initial,params.nbins(1),params.nbins(2)));hold on
if map_nbr~=0
    patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
end
hold on
plot(real_source_position(2),real_source_position(1),'p','MarkerSize',20,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
hold on
plot(1.5,3,'ko','MarkerSize',12,'MarkerFaceColor','m');
hold on
%Save the figure
axis equal; 
view(180,90);
title(['     PDF   ']);
xlabel('Y(m)'); ylabel('X(m)');     load MyColormap2; colormap(mymap); 
colorbar; caxis([0 0.002]);
hold off;
xlim([0 4])
ylim([2 13])
ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
ax.TitleFontSizeMultiplier = 1.5;
img_filename = [anim_folder, '/', num2str(0), '.png'];
try
    saveas(f2, img_filename);
    disp(['Image ', num2str(0), ' saved successfully.']);
catch
    disp(['Error: Image ', num2str(0), ' not saved. Check the folder permissions or file path.']);
end




% Close the figure 
% close(f1);
% close all

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
