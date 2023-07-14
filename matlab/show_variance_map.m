clear all; close all; clc;

%general_folder='/home/wjin/Workspace/continuous_STE/Results/results_test_1_sense_reverse_model/1/';
general_folder='/home/wjin/Workspace/continuous_STE/Results/1689356794527/101';
% experiment_path = uigetdir(general_folder);
% experiment_file_path = [experiment_path,'/101'];
experiment_file_path = general_folder;

%general_folder='/home/wjin/Workspace/continuous_STE/Results/Experimental_Results/Results_0223/2023-02-23_15-23/pc';

gas_variance_maps = dlmread([experiment_file_path,'/gas_concentration_variance_map.csv']);
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
measurements = dlmread([experiment_file_path, '/measurements.csv']); %measurement_history
entropy = dlmread([experiment_file_path, '/entropy.csv']);
positions = dlmread([experiment_file_path, '/goal_position.csv']);
real_source_position = params.source_position;


x=linspace(2.75,12.75,20);
y=linspace(0,4,20);

%% show pdf
f =figure('Position',[601.0000  111.4000  697.6000  1450.6000]);
for i=1:size(pdf,1)
    imagesc(y,x,reshape(pdf(i,:),20,20))
    hold on
%     patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
    axis equal; axis([-0.1,4.1,2.4,12.6]);view(180,90);
    title(['Posterior pdf on X and Y \newline           iteration ', num2str(i)]);
    xlabel('Y(m)'); ylabel('X(m)');
    load MyColormap2; colormap(mymap); colorbar;caxis([0 Inf]);
    hold off;
    xlim([0 4])
    ylim([2 13])
    ax = gca; ax.XDir = 'reverse'; ax.YDir = 'normal';
    ax = gca;
    ax.TitleFontSizeMultiplier = 3;
    pause;
    drawnow
%     saveas(f, [anim_folder,'/',num2str(i),'.png']);
end
%% show dispersion map
f =figure('Position',[601.0000  111.4000  697.6000  1450.6000]);
for i=1:size(gas_variance_maps,1)
    imagesc(y,x,reshape(gas_variance_maps(i,1:end-1),20,20))
    hold on
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
    hold on
    plot(positions(i,3),positions(i,2),'ko','MarkerSize',25,'MarkerFaceColor','m');
    hold on
    axis equal; axis([-0.1,4.1,2.4,12.6]);view(180,90);
    title(['Posterior pdf on X and Y \newline           iteration ', num2str(i)]);
    xlabel('Y(m)'); ylabel('X(m)');
    load MyColormap2; colormap(mymap); colorbar;caxis([0 Inf]);
    hold off;
    xlim([0 4])
    ylim([2 13])
    ax = gca; ax.XDir = 'reverse'; ax.YDir = 'normal';
    ax = gca;
    ax.TitleFontSizeMultiplier = 3;
    pause;
    drawnow
%     saveas(f, [anim_folder,'/',num2str(i),'.png']);
end