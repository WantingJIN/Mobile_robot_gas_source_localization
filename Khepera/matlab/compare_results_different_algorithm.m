% Create an empty table to store experimental data
clear all; clc; close all;
dataTable = table();

%% load the data
% the general folder
% general_folder_list = ["/home/wjin/Workspace/continuous_STE/Results/ICRA2024/0816_informative_KLD_stop_sense_MOX",...
%     "/home/wjin/Workspace/continuous_STE/Results/ICRA2024/0816_informative_KLD_continunous_MOX",...
%     "/home/wjin/Workspace/continuous_STE/Results/ICRA2024/0816_informative_KLD_visit_sense_MOX"];
general_folder_list = ["/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/clustering_continunous_MOX",...
    "/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/KLD_stop_sense_MOX"];
num_comp = size(general_folder_list,2);

%% load all the data and combine the results
for i = 1:num_comp
    tbl = readtable(strcat(general_folder_list(i),"/experiment_data.csv"));
    dataTable = [dataTable; tbl];
end

%% box plot of comparing the method
f = figure('Position',[-1561         656         1215         615]);
sgtitle("Comparison between stop/visit-sense-go and sample in motion with MOX sensor", 'FontSize',20)
mapOrder = [1, 2, 3, 5];
dataTable.Map = categorical(dataTable.Map,mapOrder);
subplot(1,5,1)
boxchart(dataTable.Map,dataTable.x_error,'GroupByColor',dataTable.planning_mode,'BoxWidth',0.55)
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([-1,10])
ylabel('(m)')
title('Error in x (m)')
legend
subplot(1,5,2)
boxchart(dataTable.Map,dataTable.y_error,'GroupByColor',dataTable.planning_mode)
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([-2,2])
ylabel('(m)')
title('Error in y (m)')
legend
subplot(1,5,3)
boxchart(dataTable.Map,dataTable.iteration,'GroupByColor',dataTable.planning_mode)
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([0,35])
title('Iterations')
legend
subplot(1,5,4)
boxchart(dataTable.Map,dataTable.exp_time,'GroupByColor',dataTable.planning_mode)
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylabel('(s)')
title('Experiment time')
legend
subplot(1,5,5)
boxchart(dataTable.Map,dataTable.traj_length,'GroupByColor',dataTable.planning_mode)
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylabel('(m)')
title('Trajectory length')
legend
