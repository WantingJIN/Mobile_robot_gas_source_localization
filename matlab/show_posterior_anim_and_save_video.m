% Plots robot trajectory with collected measurements, posterior PDF map and
% navigation planning results
clc; clear all

general_folder='/home/wanting/Workspace/kheperaodor/plume_model_learning/Results/ADE_relative_2nd_test/0';
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
    cluster_bounding_boxes = dlmread([experiment_file_path, '/clustering_bounding_boxes.csv'])
end
real_source_position = params.source_position;

if (params.planning_strategy == 5)
    gas_information_map = dlmread([experiment_file_path,'/posterior_pdf_3D.csv']);
end 

x=linspace(3,12.75,20);
y=linspace(0,4,20);

trajectory = dlmread([experiment_file_path, '/trajectory.csv']);
trajectory = [positions(1,:); trajectory];

%% show measurement
 f1 =figure('Position',[301.0000  100.4000  1500.6000  800.6000]);


%% save it to a video
v = VideoWriter([experiment_path, '/pdf.avi']);  % Output file name (can also use .mp4)
v.FrameRate = 10;                % Adjust frame rate (frames per second)
open(v);

%% show pdf
for i=1:size(pdf,1)
%     subplot(1,3,2)
    imagesc(y,x,reshape(pdf(i,:),params.nbins(1),params.nbins(2)));
    if map_nbr~=0
        patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
    end
    hold on
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    plot(positions(i,3),positions(i,2),'ko','MarkerSize',25,'MarkerFaceColor','m');
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
    xlim([0 4])
    ylim([2 13])
    ax = gca; ax.XDir = 'normal'; ax.YDir = 'reverse';
    axis equal; axis([-0.1,4.1,2.4,12.6]);
    view(180,90);  
    load MyColormap2; colormap(mymap); 
    title(['Posterior pdf on X and Y \newline           iteration ', num2str(i)]);
    ax.TitleFontSizeMultiplier = 2;
    

    drawnow
    frame = getframe(gca);  % Capture the current figure
    writeVideo(v, frame);   % Write the frame to the video

    % Save the figure
%     img_filename = [anim_folder, '/', num2str(i), '.png'];
%     try
%         saveas(f1, img_filename);
%         disp(['Image ', num2str(i), ' saved successfully.']);
%     catch
%         disp(['Error: Image ', num2str(i), ' not saved. Check the folder permissions or file path.']);
%     end


end
