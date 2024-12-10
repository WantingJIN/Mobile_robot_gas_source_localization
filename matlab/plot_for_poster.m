% Create an empty table to store experimental data
clear all; clc; close all;

plot_folder='../Results/Archive/ICRA2024/Final_campaign/Poster_plot';

%% PID: stop-sense-go vs visit sense go
% load the data
% the general folder
dataTable = table();
general_folder_list = ["../Results/Archive/ICRA2024/Final_campaign/PID/KLD_visit_sense_PID", ...
    "../Results/Archive/ICRA2024/Final_campaign/PID/KLD_stop_sense_PID"];

num_comp = size(general_folder_list,2);

% load all the data and combine the results
for i = 1:num_comp
    tbl = readtable(strcat(general_folder_list(i),"/experiment_data.csv"));
    dataTable = [dataTable; tbl];
end


% MOX: stop sense go with visit sense go
% f = figure('Position',[-1561         656         1215         615]);
f = figure('Position',[-1561         656         350         260]);
[ha, pos] = tight_subplot(1,2,[.2 .06],[.13 .1],[.04 .01])
%sgtitle("Comparison between stop/visit-sense-go and sample in motion with MOX sensor", 'FontSize',15)
%sgtitle("Comparison between informative kld and belief clustering with continuous MOX sensor", 'FontSize',15)
%samplingModeOrder = ["stop sense go", "visit sense go"];
samplingModeOrder = ["stop sense go", "visit sense go"];
dataTable.sampling_mode = categorical(dataTable.sampling_mode, samplingModeOrder);
groupcat = dataTable.sampling_mode;
mapOrder = [1];
dataTable.Map = categorical(dataTable.Map,mapOrder);
%subplot(1,3,1)
axes(ha(1));
boxChart = boxchart(dataTable.Map,dataTable.estimate_error,'GroupByColor',groupcat,'BoxWidth',0.5);
colororder(["#EDB120", "#77AC30"])
% colororder(["#89ABE3"; "#EA738D"])
% colororder(["#FF8000";"#006633"; "#0072BD"])
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
set(gca,'XTickLabel',{'1'});
grid on;
yticks(0:2:10);
ylim([-1,10])
xlabel('Map id','FontSize', 12)
title('Estimation error [m]','FontSize', 12)
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
xlabel('Map id','FontSize', 12)
title('Iterations','FontSize', 12)
legend('FontSize',12)
saveas(f,[plot_folder '/PID_stop_Vs_visit_sense.png']);
saveas(f,[plot_folder '/PID_stop_Vs_visit_sense.eps'],'epsc');

%% MOX: stop sense go with visit sense go
% load the data
% the general folder
dataTable = table();
general_folder_list = ["../Results/Archive/ICRA2024/Final_campaign/MOX/KLD_visit_sense_MOX", ...
    "../Results/Archive/ICRA2024/Final_campaign/MOX/KLD_stop_sense_MOX"];

num_comp = size(general_folder_list,2);

% load all the data and combine the results
for i = 1:num_comp
    tbl = readtable(strcat(general_folder_list(i),"/experiment_data.csv"));
    dataTable = [dataTable; tbl];
end


% MOX: stop sense go with visit sense go
% f = figure('Position',[-1561         656         1215         615]);
f = figure('Position',[-1561         656         360         260]);
[ha, pos] = tight_subplot(1,2,[.2 .06],[.13 .1],[.04 .01])
%sgtitle("Comparison between stop/visit-sense-go and sample in motion with MOX sensor", 'FontSize',15)
%sgtitle("Comparison between informative kld and belief clustering with continuous MOX sensor", 'FontSize',15)
%samplingModeOrder = ["stop sense go", "visit sense go"];
samplingModeOrder = ["stop sense go", "visit sense go"];
dataTable.sampling_mode = categorical(dataTable.sampling_mode, samplingModeOrder);
groupcat = dataTable.sampling_mode;
mapOrder = [1];
dataTable.Map = categorical(dataTable.Map,mapOrder);
%subplot(1,3,1)
axes(ha(1));
boxChart = boxchart(dataTable.Map,dataTable.estimate_error,'GroupByColor',groupcat,'BoxWidth',0.5);
colororder(["#0072BD", "#D95319"])
% colororder(["#89ABE3"; "#EA738D"])
% colororder(["#FF8000";"#006633"; "#0072BD"])
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
set(gca,'XTickLabel',{'1'});
grid on;
yticks(0:2:10);
ylim([-1,10])
xlabel('Map id','FontSize', 12)
title('Estimation error [m]','FontSize', 12)
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
xlabel('Map id','FontSize', 12)
title('Iterations','FontSize', 12)
legend('FontSize',12)
saveas(f,[plot_folder '/MOX_stop_Vs_visit_sense.png']);
saveas(f,[plot_folder '/MOX_stop_Vs_visit_sense.eps'],'epsc');
%------------------------------------------------------------------------------------------------------------%
%% MOX: stop sense go with sense in motion
dataTable = table();
general_folder_list = ["../Results/Archive/ICRA2024/Final_campaign/MOX/KLD_stop_sense_MOX", ...
    "../Results/Archive/ICRA2024/Final_campaign/MOX/KLD_continunous_MOX"];

num_comp = size(general_folder_list,2);

% load all the data and combine the results
for i = 1:num_comp
    tbl = readtable(strcat(general_folder_list(i),"/experiment_data.csv"));
    dataTable = [dataTable; tbl];
end


f = figure('Position',[-1561         656         360         260]);
[ha, pos] = tight_subplot(1,2,[.2 .06],[.13 .1],[.04 .01])
%sgtitle("Comparison between stop/visit-sense-go and sample in motion with MOX sensor", 'FontSize',15)
%sgtitle("Comparison between informative kld and belief clustering with continuous MOX sensor", 'FontSize',15)
mapOrder = [1];
dataTable.Map = categorical(dataTable.Map,mapOrder);
samplingModeOrder = ["stop sense go", "sense in motion"];
dataTable.sampling_mode = categorical(dataTable.sampling_mode, samplingModeOrder);
groupcat = dataTable.sampling_mode;
%subplot(1,3,1)
axes(ha(1));
boxChart = boxchart(dataTable.Map,dataTable.estimate_error,'GroupByColor',groupcat,'BoxWidth',0.5);
colororder(["#0072BD", 	"#7E2F8E"])
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
set(gca,'XTickLabel',{'1'});
grid on;
yticks(0:2:10);
ylim([-1,10])
xlabel('Map id','FontSize', 12)
title('Estimation error [m]','FontSize', 12)
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
xlabel('Map id','FontSize', 12)
title('Iterations','FontSize', 12)
legend('FontSize',12)
saveas(f,[plot_folder '/MOX_stop_Vs_motion_sense.png']);
saveas(f,[plot_folder '/MOX_stop_Vs_motion_sense.eps'],'epsc');
%% MOX with sense-in-motion: informative kld with belief clustering
dataTable = table();
general_folder_list = ["../Results/Archive/ICRA2024/Final_campaign/New_MOX/KLD_continunous_new_MOX",...
    "../Results/Archive/ICRA2024/Final_campaign/New_MOX/clustering_continuous_sense_new_MOX"];
num_comp = size(general_folder_list,2);

% load all the data and combine the results
for i = 1:num_comp
    tbl = readtable(strcat(general_folder_list(i),"/experiment_data.csv"));
    dataTable = [dataTable; tbl];
end

f = figure('Position',[-1561         656         360         260]);
[ha, pos] = tight_subplot(1,2,[.2 .06],[.13 .1],[.04 .01])

mapOrder = [1];
dataTable.Map = categorical(dataTable.Map,mapOrder);

planningModeOrder = ["informative kld", "belief clustering"];
dataTable.plannin_mode= categorical(dataTable.planning_mode, planningModeOrder);

groupcat = dataTable.planning_mode;
%subplot(1,3,1)
axes(ha(1));
boxChart = boxchart(dataTable.Map,dataTable.estimate_error,'GroupByColor',groupcat,'BoxWidth',0.5);
% colororder(["#0072BD", "#EDB120"])
colororder(["#89ABE3"; "#EA738D"])
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
set(gca,'XTickLabel',{'1'});
grid on;
yticks(0:2:10);
ylim([-1,10])
xlabel('Map id','FontSize', 12)
title('Estimation error [m]','FontSize', 12)
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
xlabel('Map id','FontSize', 12)
title('Iterations','FontSize', 12)
legend('FontSize',12)
saveas(f,[plot_folder '/MOX_informative_kld_Vs_belief_clustering.png']);
saveas(f,[plot_folder '/MOX_informative_kld_Vs_belief_clustering.eps'],'epsc');


%% MOX with sense-in-motion vs PID with stop-sense-go
dataTable = table();
general_folder_list = ["../Results/Archive/ICRA2024/Final_campaign/PID/KLD_stop_sense_PID",...
    "../Results/Archive/ICRA2024/Final_campaign/New_MOX/KLD_continunous_new_MOX"];
num_comp = size(general_folder_list,2);

% load all the data and combine the results
for i = 1:num_comp
    tbl = readtable(strcat(general_folder_list(i),"/experiment_data.csv"));
    dataTable = [dataTable; tbl];
end

f = figure('Position',[-1561         656         720         310]);
[ha, pos] = tight_subplot(1,2,[.2 .06],[.13 .1],[.04 .01])

mapOrder = [1];
dataTable.Map = categorical(dataTable.Map,mapOrder);

samplingModeOrder = ["stop sense go", "visit sense go", "sense in motion"];
dataTable.sampling_mode = categorical(dataTable.sampling_mode, samplingModeOrder);

groupcat = dataTable.sampling_mode;
%subplot(1,3,1)
axes(ha(1));
boxChart = boxchart(dataTable.Map,dataTable.estimate_error,'GroupByColor',groupcat,'BoxWidth',0.5);
colororder(["#0072BD", 	"#7E2F8E"])
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
% set(gca,'XTickLabel',{'1'});
grid on;
yticks(0:2:10);
ylim([-1,10])
% xlabel('Map id','FontSize', 12)
title('Estimation error [m]','FontSize', 14)
legend(["stop sense go with PID sensor","sense in motion with MOX sensor"],'FontSize',14)
%subplot(1,3,2)
axes(ha(2));
boxchart(dataTable.Map,dataTable.iteration,'GroupByColor',groupcat)
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
% set(gca,'XTickLabel',{'1'});
grid on;
ylim([0,40])
%xlabel('Map id','FontSize', 12)
title('Iterations','FontSize', 14)
legend(["stop sense go with PID sensor","sense in motion with MOX sensor"],'FontSize',14)
saveas(f,[plot_folder '/MOX_continuous_sense_Vs_PID_stop_sense.png']);
saveas(f,[plot_folder '/MOX_continuous_sense_Vs_PID_stop_sense.eps'],'epsc');


