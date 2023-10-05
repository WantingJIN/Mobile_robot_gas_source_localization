% Create an empty table to store experimental data
clear all; clc; close all;
dataTable = table();

plot_folder='/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/Plot';

%% load the data
% the general folder
general_folder_list = ["/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/MOX/KLD_continunous_MOX",...
    "/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/MOX/KLD_visit_sense_MOX", ...
    "/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/MOX/KLD_stop_sense_MOX"];
% general_folder_list = ["/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/MOX/KLD_continunous_MOX",...
%     "/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/MOX/clustering_MOX_continuous_new_cluster_selection"];
%  general_folder_list = ["/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/PID/KLD_visit_sense_PID",...
%      "/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/PID/KLD_stop_sense_PID"];
num_comp = size(general_folder_list,2);

%% load all the data and combine the results
for i = 1:num_comp
    tbl = readtable(strcat(general_folder_list(i),"/experiment_data.csv"));
    dataTable = [dataTable; tbl];
end

%% box plot of comparing the method
% f = figure('Position',[-1561         656         1215         615]);
% [ha, pos] = tight_subplot(1,3,[.2 .04],[.06 .1],[.03 .01])
% %sgtitle("Comparison between stop/visit-sense-go and sample in motion with MOX sensor", 'FontSize',15)
% %sgtitle("Comparison between informative kld and belief clustering with continuous MOX sensor", 'FontSize',15)
% samplingModeOrder = ["stop sense go", "visit sense go", "sense in motion"];
% dataTable.sampling_mode = categorical(dataTable.sampling_mode, samplingModeOrder);
% planningModeOrder = ["informative kld", "belief clustering"];
% dataTable.planning_mode = categorical(dataTable.planning_mode, planningModeOrder);
% groupcat = dataTable.sampling_mode;
% mapOrder = [0, 1, 2, 3, 5,10];
% dataTable.Map = categorical(dataTable.Map,mapOrder);
% %subplot(1,3,1)
% axes(ha(1));
% boxChart = boxchart(dataTable.Map,dataTable.x_error,'GroupByColor',groupcat,'BoxWidth',0.5);
% %colororder(["#EDB120", "#77AC30"])
% %colororder(["#89ABE3"; "#EA738D"])
% colororder(["#FF8000";"#006633"; "#0072BD"])
% lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
% set(lines, 'Color', 'r');
% set(findobj(gca,'type','line'),'linew',1.2)
% grid on;
% ylim([-1,10])
% %ylabel('(m)')
% title('Error in x (m)')
% legend
% %subplot(1,3,2)
% axes(ha(2));
% boxchart(dataTable.Map,dataTable.y_error,'GroupByColor',groupcat)
% lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
% set(lines, 'Color', 'r');
% set(findobj(gca,'type','line'),'linew',1.2)
% grid on;
% ylim([-2,2])
% %ylabel('(m)')
% title('Error in y (m)')
% legend
% %subplot(1,3,3)
% axes(ha(3));
% boxchart(dataTable.Map,dataTable.iteration,'GroupByColor',groupcat)
% lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
% set(lines, 'Color', 'r');
% set(findobj(gca,'type','line'),'linew',1.2)
% grid on;
% ylim([0,40])
% title('Iterations')
% legend
% subplot(1,5,4)
% boxchart(dataTable.Map,dataTable.exp_time,'GroupByColor',groupcat)
% lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
% set(lines, 'Color', 'r');
% set(findobj(gca,'type','line'),'linew',1.2)
% grid on;
% ylabel('(s)')
% title('Experiment time')
% legend
% subplot(1,5,5)
% boxchart(dataTable.Map,dataTable.traj_length,'GroupByColor',groupcat)
% lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
% set(lines, 'Color', 'r');
% set(findobj(gca,'type','line'),'linew',1.2)
% grid on;
% ylabel('(m)')
% title('Trajectory length')
% legend
%% box plot of comparing the method
% f = figure('Position',[-1561         656         1215         615]);
f = figure('Position',[-1561         656         915         615]);
[ha, pos] = tight_subplot(1,2,[.2 .06],[.13 .1],[.04 .01])
%sgtitle("Comparison between stop/visit-sense-go and sample in motion with MOX sensor", 'FontSize',15)
%sgtitle("Comparison between informative kld and belief clustering with continuous MOX sensor", 'FontSize',15)
%samplingModeOrder = ["stop sense go", "visit sense go"];
samplingModeOrder = ["stop sense go", "visit sense go", "sense in motion"];

dataTable.sampling_mode = categorical(dataTable.sampling_mode, samplingModeOrder);
planningModeOrder = ["informative kld", "belief clustering"];
dataTable.planning_mode = categorical(dataTable.planning_mode, planningModeOrder);
groupcat = dataTable.sampling_mode;
% mapOrder = [0, 1, 2, 3, 5,10];
mapOrder = 1;
dataTable.Map = categorical(dataTable.Map,mapOrder);
%subplot(1,3,1)
axes(ha(1));
boxChart = boxchart(dataTable.Map,dataTable.estimate_error,'GroupByColor',groupcat,'BoxWidth',0.5);
%colororder(["#EDB120", "#77AC30"])
% colororder(["#89ABE3"; "#EA738D"])
colororder(["#FF8000";"#006633"; "#0072BD"])
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
set(gca,'XTickLabel',{'1'});
grid on;
yticks(0:2:10);
ylim([-1,10])
xlabel('Map id')
title('Estimation error (m)')
legend('FontSize',12)
%subplot(1,3,2)
axes(ha(2));
boxchart(dataTable.Map,dataTable.iteration,'GroupByColor',groupcat)
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
set(gca,'XTickLabel',{'1'});
grid on;
ylim([0,40])
xlabel('Map id')
title('Iterations')
legend('FontSize',12)
saveas(f,[plot_folder '/MOX_continous_KLD_Vs_cluster.png']);
saveas(f,[plot_folder '/MOX_continous_KLD_Vs_cluster.eps'],'epsc');
% subplot(1,5,4)
% boxchart(dataTable.Map,dataTable.exp_time,'GroupByColor',groupcat)
% lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
% set(lines, 'Color', 'r');
% set(findobj(gca,'type','line'),'linew',1.2)
% grid on;
% ylabel('(s)')
% title('Experiment time')
% legend
% subplot(1,5,5)
% boxchart(dataTable.Map,dataTable.traj_length,'GroupByColor',groupcat)
% lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
% set(lines, 'Color', 'r');
% set(findobj(gca,'type','line'),'linew',1.2)
% grid on;
% ylabel('(m)')
% title('Trajectory length')
% legend
