% While calibrate sensor reading using the experiments data
% experiment_folder = '/home/durancea/plume_model_learning/Results/Experimental_Results/Results-0305/2023-03-05_11-14/';
% experiment_measurement = dlmread([experiment_folder 'k4/measurements.csv']);
% baseline_measurement = dlmread([experiment_folder 'k4/baseline_measurements.csv']);
% webots_ground_truth = dlmread([experiment_folder 'Ground_truth_wb/measurements.csv']);
% experiment_gas_con = experiment_measurement(:,3) - baseline_measurement(1,3);

close all

% While calibrating sensor reading using scan file
experiment_folder = '/home/wjin/Workspace/Wind_tunnel_scan/result_0510/05-10_BoardC_StopSense_PumpOn/log_K4_scan/';
experiment_measurement = dlmread([experiment_folder 'measurements.csv']);
experiment_gas_con = experiment_measurement(:,3);
webots_gt_folder = [experiment_folder '/Ground_truth_wb/101/'];
webots_gt_measurment = dlmread([webots_gt_folder 'measurements.csv']);
webots_gas_con = webots_gt_measurment(:,3);

positions = dlmread([experiment_folder 'position_history.csv']);
figure()
subplot(2,1,1)
scatter(positions(:,2), positions(:,3), 30, experiment_gas_con, 'filled');
clim([0, 5000]);
load MyColormap.mat; colormap(mymap); colorbar;
axis equal;
axis([4 12.7 0 4]);
title("Experiment data in wt")
grid on;
subplot(2,1,2)
scatter(positions(:,2), positions(:,3), 30, webots_gas_con, 'filled');
clim([0, 15000]);
load MyColormap.mat; colormap(mymap); colorbar;
axis equal;
axis([4 12.7 0 4]);
title("Webots ground truth")
grid on;
saveas(gcf,[experiment_folder 'webots_experiment_compare'  '.png']);

%% compensent the tailtored effect of the wind in the wt
y_baseline = 5;
y_step = 1;
j = 1;
experiment_gas_in_line = {};
webots_gas_in_line = {}; 
experiment_gas_in_line{1} = experiment_gas_con(1);
webots_gas_in_line{1} = webots_gas_con(1);
% group the measurement for each line
for i = 1:size(experiment_gas_con, 1)
    if (positions(i, 2) < y_baseline + y_step * j)      
         experiment_gas_in_line{j} = [experiment_gas_in_line{j}, experiment_gas_con(i)];
         webots_gas_in_line{j} = [webots_gas_in_line{j}, webots_gas_con(i)];
    else
        j = j + 1;
        experiment_gas_in_line{j} = experiment_gas_con(i);
        webots_gas_in_line{j} = webots_gas_con(i);
    end
end
max_line = j;
experiment_valid_gas_reading = [];
webots_valid_gas_reading = [];
% for each line, extract the top three value (with the close index)
for i = 1 : size(experiment_gas_in_line,2)
    [maxValue, maxIndex] = max(experiment_gas_in_line{i});
    experiment_valid_gas_reading(end+1) = experiment_gas_in_line{i}(maxIndex-1);
    experiment_valid_gas_reading(end+1) = experiment_gas_in_line{i}(maxIndex);
    experiment_valid_gas_reading(end+1) = experiment_gas_in_line{i}(maxIndex+1);

    [maxValue, maxIndex] = max(webots_gas_in_line{i});
    webots_valid_gas_reading(end+1) = webots_gas_in_line{i}(maxIndex-1);
    webots_valid_gas_reading(end+1) = webots_gas_in_line{i}(maxIndex);
    webots_valid_gas_reading(end+1) = webots_gas_in_line{i}(maxIndex+1);
end

        
gas_concentration_compare=[];
gas_concentration_compare(1,:) = webots_valid_gas_reading;
gas_concentration_compare(2,:) = experiment_valid_gas_reading;
gas_concentration_compare(3,:) = webots_valid_gas_reading - experiment_valid_gas_reading; 

gas_concentration_compare'


% 
% for i = 1: size(experiment_gas_con)
%     if experiment_gas_con(i,1) <0
%         experiment_gas_con(i,1) =0;
%     end
%     if experiment_gas_con(i,1) == 0
%         webots_gas_con(i,1)=0;
%     end
% end

% find the coorelation between two groups of data
figure()
plot(experiment_valid_gas_reading, webots_valid_gas_reading, 'o')
hold on
s = fitoptions('Method','NonlinearLeastSquares',...
               'Lower',[0,0],...
               'Startpoint',[1 1]);
f = fittype('a*(x-b)^n','problem','n','options',s);
[c2,gof2] = fit(experiment_valid_gas_reading',webots_valid_gas_reading',f,'problem',1)
plot(c2,'m')

