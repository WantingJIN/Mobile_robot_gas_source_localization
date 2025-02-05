% Plots robot trajectory with visited cells map

clear all; close all; clc;

general_folder='/home/durancea/plume_model_learning/Results/1703671246493/101';
%experiment_path = uigetdir(general_folder);
%experiment_file_path = [experiment_path,'/101'];
experiment_file_path = general_folder;

clustering=0
pdf = dlmread([experiment_file_path, '/posterior_pdf_3D.csv']);
visited_list = dlmread([experiment_file_path, '/visited_list.csv']);
map_folder=[general_folder '/map'];
params = my_text2struct([experiment_file_path,'/param_log.csv']); 
map_nbr = params.map_nbr;
if map_nbr~=0
    obst_file = ['/home/durancea/plume_model_learning/Khepera/Data/Obstacle_lists/' num2str(map_nbr) '_obstacles.txt'];
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
if clustering~=0
    cluster_bounding_boxes = dlmread([experiment_file_path, '/clustering_bounding_boxes.csv'])
end
real_source_position = params.source_position;

x=linspace(3,12.75,20);
y=linspace(0,4,20);

trajectory = dlmread([experiment_file_path, '/trajectory.csv']);
trajectory = [positions(1,:); trajectory];
gas_feature = params.gas_feature;

test = imagesc(y,x,reshape(visited_list(1,1:end-1),params.nbins(1),params.nbins(2)))


%% show visited list
f =figure('Position',[601.0000  111.4000  697.6000  1450.6000]);
imagesc(y,x,reshape(visited_list(1,1:end-1),params.nbins(1),params.nbins(2)));
% f =figure(test);
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
colorbar;caxis([0 Inf]);
hold off;
xlim([0 4])
ylim([2 13])
ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
title(['Trajectory and Measurements']);
ax.TitleFontSizeMultiplier = 2;
drawnow

% Save the figure
img_filename = [anim_folder, '/visited.png'];
try
    saveas(f, img_filename);
    disp(['Image ', ' saved successfully.']);
catch
    disp(['Error: Image ', ' not saved. Check the folder permissions or file path.']);
end
% Close the figure 
%close(f);
%close all

