%% read experiment files
clear all; close all; clc;
%general_folder='/home/wjin/Workspace/continuous_STE/Results/results_test_1_sense_reverse_model/1/';
general_folder='/home/wjin/Workspace/continuous_STE/Results/0802_test_informative_clustering/1/1690971208394/101';
% experiment_path = uigetdir(general_folder);
% experiment_file_path = [experiment_path,'/101'];
experiment_file_path = general_folder;

%general_folder='/home/wjin/Workspace/continuous_STE/Results/Experimental_Results/Results_0223/2023-02-23_15-23/pc';

gas_maps = dlmread([experiment_file_path,'/gas_concentration_map.csv']);
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
%% plot gas concentration map
f =figure('Position',[601.0000  111.4000  697.6000  1450.6000]);
for i=1:size(gas_maps,1)
    source_position = gas_maps(i,1:2); 
    % be careful: the reshape functio will first fill the column and then
    % row: Use the revert matrix!!
    imagesc(y,x,reshape(gas_maps(i,3:end),20,20)')
    hold on
    plot(source_position(2),source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
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
%% plot pdf
% close all
% f =figure('Position',[601.0000  111.4000  697.6000  1450.6000]);
% for i=1:size(pdf,1)
%     imagesc(y,x,reshape(pdf(i,:),20,20))
%     hold on
% %     patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
%     axis equal; axis([-0.1,4.1,2.4,12.6]);view(180,90);
%     title(['Posterior pdf on X and Y \newline           iteration ', num2str(i)]);
%     xlabel('Y(m)'); ylabel('X(m)');
%     load MyColormap2; colormap(mymap); colorbar;caxis([0 Inf]);
%     hold off;
%     xlim([0 4])
%     ylim([2 13])
%     %ax = gca; ax.XDir = 'reverse'; ax.YDir = 'normal';
%     ax = gca;
%     ax.TitleFontSizeMultiplier = 3;
%     pause;
%     drawnow
% %     saveas(f, [anim_folder,'/',num2str(i),'.png']);
% end


%% calculate average value of gas concentration
f =figure('Position',[601.0000  111.4000  697.6000  1450.6000]);
gas_concentration_sum = zeros(1,400);
for i = 1:size(gas_maps,1)
    gas_concentration_sum = gas_concentration_sum + gas_maps(i,3:end);
end
gas_concentration_avg = gas_concentration_sum/size(gas_maps,1);
imagesc(y,x,reshape(gas_concentration_avg,20,20)')
axis equal;view(180,90);axis([-0.1,4.1,2.4,12.6])
%% calculate the gas variance for uniform source position distribution
f =figure('Position',[601.0000  111.4000  697.6000  1450.6000]);
gas_variance_sum = zeros(1,400);

for i = 1:size(gas_maps,1)
    gas_variance_sum = gas_variance_sum + (gas_maps(i,3:end)-gas_concentration_avg).^2;
end
gas_concentration_std = sqrt(gas_variance_sum)/size(gas_maps,1);

imagesc(y,x,reshape(gas_concentration_std,20,20)')
axis equal;view(180,90);axis([-0.1,4.1,2.4,12.6])

%% calculate the weighted gas average value
f =figure('Position',[601.0000  111.4000  697.6000  1450.6000]);
for iter = 1:size(pdf,1)
    subplot(1,2,1)
    imagesc(y,x,reshape(pdf(iter,:),20,20));
    axis equal;view(180,90);axis([-0.1,4.1,2.4,12.6])
    xlim([0 4])
    ylim([2 13])
    subplot(1,2,2)
    gas_weighted_sum = zeros(1,400);
    title("pdf")
    for i = 1:size(gas_maps,1)
        gas_weighted_sum = gas_weighted_sum + gas_maps(i,3:end)*pdf(iter,i);
    end
    imagesc(y,x,reshape(gas_weighted_sum,20,20)')
    axis equal;view(180,90);axis([-0.1,4.1,2.4,12.6])
    xlim([0 4])
    ylim([2 13])
    ax = gca;
    title('weighted expected concentration')
    pause;
    drawnow
end

%% calculate the weighted gas variance by taking into account possility distribution 
for iter = 1:size(pdf,1)
    gas_weighted_sum = zeros(1,400);
    gas_weighted_variance_sum = zeros(1,400);
    for i = 1:size(gas_maps,1)
        gas_weighted_sum = gas_weighted_sum + gas_maps(i,3:end)*pdf(iter,i);
    end
    for i = 1:size(gas_maps,1)
        gas_weighted_variance_sum = gas_weighted_variance_sum + (gas_maps(i,3:end)-gas_weighted_sum).^2*pdf(iter,i);
    end
    gas_weighted_concentration_std = sqrt(gas_weighted_variance_sum);
    % normalize the variance map
%     max_var = max(gas_weighted_concentration_std);
%     min_var = min(gas_weighted_concentration_std);
%     gas_weighted_concentration_std = (gas_weighted_concentration_std - min_var)/(max_var - min_var);
%     gas_weighted_concentration_std = reshape(gas_weighted_concentration_std,20,20);
%     gas_variance_with_idx=[];
%     for row = 1:20
%         for col = 1:20
%            gas_variance_with_idx=[gas_variance_with_idx; x(row), y(col), gas_weighted_concentration_std(row,col)];
%         end
%     end
% 
%     index = gas_variance_with_idx(:,3) > 0.5;
%     gas_variance_with_idx = gas_variance_with_idx(index,:);
%     figure();
%     scatter(gas_variance_with_idx(:,1),gas_variance_with_idx(:,2),'.');
% %     Set the parameters for DBSCAN
%     epsilon = 0.1; % Maximum distance between two points to be considered neighbors
%     minPts = 1;   % Minimum number of points required to form a cluster
%     figure();
%      % Apply DBSCAN
%     D = pdist2(gas_variance_with_idx,gas_variance_with_idx);
%     [idx, corepts] = dbscan(D,0.1,2,'Distance','precomputed');
%     numGroups = length(unique(idx));
%     gscatter(gas_variance_with_idx(:,1),gas_variance_with_idx(:,2),idx,hsv(numGroups));
%     grid
    
    imagesc(y,x,reshape(gas_weighted_concentration_std,20,20)');
    hold on
    plot(positions(iter,3),positions(iter,2),'ko','MarkerSize',25,'MarkerFaceColor','m');
    hold on
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    axis equal;view(180,90);axis([-0.1,4.1,2.4,12.6]);
    xlim([0 4]);
    ylim([2 13]);
    ax = gca;
    pause;
    drawnow
end

