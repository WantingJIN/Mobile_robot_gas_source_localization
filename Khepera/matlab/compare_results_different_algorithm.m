% Create an empty table to store experimental data
clear all; clc; close all;
dataTable = table();

%% load the data
% the general folder
general_folder_list = ["/home/wjin/Workspace/continuous_STE/Khepera/ICRA_2024_Results/continuous_clustering_ratio10",...
    "/home/wjin/Workspace/continuous_STE/Khepera/ICRA_2024_Results/stop_sense_kld"];



num_comp = size(general_folder_list,2);

%% load all the data and combine the results
for i = 1:num_comp
    tbl = readtable(strcat(general_folder_list(i),"/experiment_data.csv"));
    dataTable = [dataTable; tbl];
end

%% box plot of comparing the method
f = figure('Position',[-1561         656         1815         615]);
[ha, pos] = tight_subplot(1,5,[.2 .04],[.15 .1],[.03 .01])
%sgtitle("Physical Experiments", 'FontSize',20)
mapOrder = [2 , 5, 10];
dataTable.Map = categorical(dataTable.Map,mapOrder);
planningModeOrder = ["informative kld", "belief clustering"];
dataTable.planning_mode = categorical(dataTable.planning_mode, planningModeOrder);
%subplot(1,5,1)
axes(ha(1))
boxchart(dataTable.Map,dataTable.x_error,'GroupByColor',dataTable.planning_mode,'BoxWidth',0.55)
colororder([0.8500 0.3250 0.0980; 0 0.4470 0.7410])
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
set(gca,'XTickLabel',{'2', '4', '5'});
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'off';
ylim([-1,10])
%ylabel('(m)')
title('Error in x [m]')
legend(["KLD-stop-sense","cluster-motion-sense"],'FontSize',8)
xlabel('Map id');
%subplot(1,5,2)
axes(ha(2))
boxchart(dataTable.Map,dataTable.y_error,'GroupByColor',dataTable.planning_mode)
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
set(gca,'XTickLabel',{'2', '4', '5'});
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'off';
ylim([-2,2])
%ylabel('(m)')
title('Error in y [m]')
legend(["KLD-stop-sense","cluster-motion-sense"],'FontSize',8)
xlabel('Map id');
%subplot(1,5,3)
axes(ha(3))
boxchart(dataTable.Map,dataTable.iteration,'GroupByColor',dataTable.planning_mode)
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
set(gca,'XTickLabel',{'2', '4', '5'});
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'off';
ylim([0,40])
title('Iterations')
legend(["KLD-stop-sense","cluster-motion-sense"],'FontSize',8)
xlabel('Map id');
%subplot(1,5,4)
axes(ha(4))
boxchart(dataTable.Map,dataTable.exp_time,'GroupByColor',dataTable.planning_mode)
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
set(gca,'XTickLabel',{'2', '4', '5'});
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'off';
%ylabel('(s)')
title('Localization time [s]')
legend(["KLD-stop-sense","cluster-motion-sense"],'FontSize',8)
xlabel('Map id');

%subplot(1,5,5)
axes(ha(5))
boxchart(dataTable.Map,dataTable.traj_length,'GroupByColor',dataTable.planning_mode)
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
set(gca,'XTickLabel',{'2', '4', '5'});
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'off';
%ylabel('(m)')
title('Trajectory length [m]')
legend(["KLD-stop-sense","cluster-motion-sense"],'FontSize',8)
xlabel('Map id');

