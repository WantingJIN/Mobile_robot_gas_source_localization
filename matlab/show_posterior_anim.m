%  Plots and stores the posterior PDF map of the source location at
%  successive iterations in anim_folder

clear all; close all; clc;

%general_folder='../Results/results_test_1_sense_reverse_model/1/';
general_folder='../Results/results_continuous_MOX_no_processing_greedy/1/1687274906178/101';
% experiment_path = uigetdir(general_folder);
% experiment_file_path = [experiment_path,'/101'];
experiment_file_path = general_folder;

pdf = dlmread([experiment_file_path,'/posterior_pdf_3D.csv']);
positions = dlmread([experiment_file_path, '/goal_position.csv']);
anim_folder=[general_folder '/ani'] ;
if(~exist(anim_folder,'dir'))
    mkdir(anim_folder);
end
params = my_text2struct([experiment_file_path,'/param_log.csv']); 
map_nbr = params.map_nbr;
obst_file = ['../Khepera/Data/Obstacle_lists/' num2str(map_nbr) '_obstacles.txt'];
obst_fileID = fopen(obst_file);
C = textscan(obst_fileID,'%f %f %f %f', 'HeaderLines', 1);
for k=1:length(C{1,1})
            obstacles_x(:,k) = [C{1,1}(k) , C{1,3}(k), C{1,3}(k) , C{1,1}(k)];
            obstacles_y(:,k) = [C{1,2}(k) , C{1,2}(k), C{1,4}(k) , C{1,4}(k)];
end
measurements = dlmread([experiment_file_path, '/measurements.csv']); %measurement_history
entropy = dlmread([experiment_file_path, '/entropy.csv']);
real_source_position = params.source_position;

x=linspace(2.75,12.75,20);
y=linspace(0,4,20);

for i = 1:size(pdf, 1)
    f = figure('Position', [601.0000  111.4000  697.6000  1450.6000]);
    imagesc(y, x, reshape(pdf(i,:), 20, 20))
    hold on
    plot(positions(i,3), positions(i,2), 'ko', 'MarkerSize', 25, 'MarkerFaceColor', 'm');
    plot(real_source_position(2), real_source_position(1), 'p', 'MarkerSize', 40, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', [51/255 153/255 1]);
    hold on
    % patch(obstacles_y, obstacles_x, [0 0 0], 'DisplayName', 'Obstacle', 'LineStyle', 'none');
    axis equal; axis([-0.1, 4.1, 2.4, 12.6]); view(180,90);
    title(['Posterior pdf on X and Y \newline           iteration ', num2str(i)]);
    xlabel('Y(m)'); ylabel('X(m)');
    load MyColormap2; colormap(mymap); colorbar; caxis([0 Inf]);
    annotation(f, 'textbox', [0.254287457153379 0.025796129034299 0.104365076491284 0.0473684201637904], 'String', {'Measurement:', num2str(measurements(i,2))}, 'LineStyle', 'none', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 24);
    annotation(f, 'textbox', [0.60287457153379 0.025796129034299 0.104365076491284 0.0473684201637904], 'String', {'Entropy:', num2str(entropy(i,1))}, 'LineStyle', 'none', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 24);
    hold off;
    xlim([0 4])
    ylim([2 13])
    ax = gca; ax.XDir = 'reverse'; ax.YDir = 'normal';
    ax = gca;
    ax.TitleFontSizeMultiplier = 3;
    drawnow;
    
    % Save the figure
    img_filename = [anim_folder, '/', num2str(i), '.png'];
    try
        saveas(f, img_filename);
        disp(['Image ', num2str(i), ' saved successfully.']);
    catch
        disp(['Error: Image ', num2str(i), ' not saved. Check the folder permissions or file path.']);
    end
    
    % Close the figure to avoid memory issues
    close(f);
    
    % Check if the image file was successfully created
    if exist(img_filename, 'file')
        disp(['Image ', num2str(i), ' saved successfully.']);
    else
        disp(['Error: Image ', num2str(i), ' not saved. Check the folder permissions or file path.']);
    end
end

close all
