%% Create an empty table to store experimental data
clear all; clc; close all;
dataTable = table();
plot_folder='../Results/ICRA2024/Final_campaign/Paper_plot';
%% compare MOX sensor sample-in-motion: KLD planning VS belief clustering
% % the general folder
% general_folder_list = ["../Results/ICRA2024/Final_campaign/New_MOX/KLD_continunous_new_MOX",...
%     "../Results/ICRA2024/Final_campaign/New_MOX/clustering_continuous_sense_new_MOX"];
% num_comp = size(general_folder_list,2);
% 
% % load all the data and combine the results
% for i = 1:num_comp
%     tbl = readtable(strcat(general_folder_list(i),"/experiment_data.csv"));
%     dataTable = [dataTable; tbl];
% end
% 
% % box plot of comparing the method
% f = figure('Position',[-1561         656         1215         615]);
% [ha, pos] = tight_subplot(1,2,[.2 .04],[.12 .1],[.03 .01])
% %sgtitle("Comparison between stop/visit-sense-go and sample in motion with MOX sensor", 'FontSize',15)
% %sgtitle("Comparison between informative kld and belief clustering with continuous MOX sensor", 'FontSize',15)
% samplingModeOrder = ["stop sense go", "visit sense go", "sense in motion"];
% dataTable.sampling_mode = categorical(dataTable.sampling_mode, samplingModeOrder);
% planningModeOrder = ["informative kld", "belief clustering"];
% dataTable.planning_mode = categorical(dataTable.planning_mode, planningModeOrder);
% 
% % change the name of map 5 to 4, and map 10 to map 5 to keep consistency
% % Create a logical index for rows where Map is 5
% logicalIndex = dataTable.Map == 5;
% 
% % Replace all occurrences of 5 with 4 using logical indexing
% dataTable.Map(logicalIndex) = 4;
% 
% % Create a logical index for rows where Map is 10
% logicalIndex = dataTable.Map == 10;
% 
% % Replace all occurrences of 10 with 5 using logical indexing
% dataTable.Map(logicalIndex) = 5;
% 
% 
% groupcat = dataTable.planning_mode;
% groupcatValue = planningModeOrder;
% 
% % we want to keep more distance between different group while put the box
% % belong to the same group closer
% % Create a logical index for rows where planning_mode is "informative kld"
% offset_step = 0.4/(size(groupcatValue,2)-1);
% offset= -0.2:offset_step:0.2;
% axes(ha(1));
% for i = size (groupcatValue)
%     logicalIndex = groupcat == groupcatValue(i);
%     sub_Table = dataTable(logicalIndex, :);
%     sub_Table.Map = sub_Table.Map + offset(i);
%     boxChart = boxchart(sub_Table.Map,sub_Table.estimate_error,'BoxWidth',offset_step-0.12);
%     hold on
% end
% 
% %colororder(["#EDB120", "#77AC30"])
% colororder(["#89ABE3"; "#EA738D"])
% %colororder(["#FF8000";"#006633"; "#0072BD"])
% lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
% set(lines, 'Color', 'r');
% set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
% xposition = [0,1,2,3,4,5];
% set(gca,'Xtick',xposition);
% xlim([-0.5,5.5])
% grid on;
% yticks(0:2:10);
% ylim([-1,10]);
% xlabel('Map id','FontSize',12);
% title('Estimation error (m)','FontSize',12);
% legend(groupcatValue,'FontSize',11);
% 
% axes(ha(2));
% for i = size (groupcatValue)
%     logicalIndex = groupcat == groupcatValue(i);
%     sub_Table = dataTable(logicalIndex, :);
%     sub_Table.Map = sub_Table.Map + offset(i);
%     boxChart = boxchart(sub_Table.Map,sub_Table.iteration,'BoxWidth',offset_step-0.15);
%     hold on
% end
% lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
% set(lines, 'Color', 'r');
% set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
% grid on;
% ylim([0,40])
% xlim([-0.5,5.5])
% xlabel('Map id','FontSize',12)
% title('Iterations','FontSize',12)
% legend(groupcatValue,'FontSize',11)
% 
% saveas(f,[plot_folder '/MOX_new_continous_KLD_Vs_cluster.png']);
% saveas(f,[plot_folder '/MOX_new_continous_KLD_Vs_cluster.eps'],'epsc');


%% compare MOX sensor with KLD planning: Stop-sense-go, visit-sense-go, sense-in-motion
% % the general folder
% general_folder_list = ["../Results/ICRA2024/Final_campaign/New_MOX/KLD_continunous_new_MOX",...
%     "../Results/ICRA2024/Final_campaign/New_MOX/KLD_visit_sense_new_MOX", ...
%     "../Results/ICRA2024/Final_campaign/New_MOX/KLD_stop_sense_new_MOX"];
% num_comp = size(general_folder_list,2);
% 
% % load all the data and combine the results
% for i = 1:num_comp
%     tbl = readtable(strcat(general_folder_list(i),"/experiment_data.csv"));
%     dataTable = [dataTable; tbl];
% end
% 
% % box plot of comparing the method
% f = figure('Position',[-1561         656         1215         615]);
% [ha, pos] = tight_subplot(1,2,[.2 .04],[.12 .1],[.03 .01])
% %sgtitle("Comparison between stop/visit-sense-go and sample in motion with MOX sensor", 'FontSize',15)
% %sgtitle("Comparison between informative kld and belief clustering with continuous MOX sensor", 'FontSize',15)
% samplingModeOrder = ["stop sense go", "visit sense go", "sense in motion"];
% dataTable.sampling_mode = categorical(dataTable.sampling_mode, samplingModeOrder);
% planningModeOrder = ["informative kld", "belief clustering"];
% dataTable.planning_mode = categorical(dataTable.planning_mode, planningModeOrder);
% 
% % change the name of map 5 to 4, and map 10 to map 5 to keep consistency
% % Create a logical index for rows where Map is 5
% logicalIndex = dataTable.Map == 5;
% 
% % Replace all occurrences of 5 with 4 using logical indexing
% dataTable.Map(logicalIndex) = 4;
% 
% % Create a logical index for rows where Map is 10
% logicalIndex = dataTable.Map == 10;
% 
% % Replace all occurrences of 10 with 5 using logical indexing
% dataTable.Map(logicalIndex) = 5;
% 
% 
% groupcat = dataTable.sampling_mode;
% groupcatValue = samplingModeOrder;
% 
% % we want to keep more distance between different group while put the box
% % belong to the same group closer
% % Create a logical index for rows where planning_mode is "informative kld"
% offset_step = 0.4/(size(groupcatValue,2)-1);
% offset= -0.25:offset_step:0.25;
% axes(ha(1));
% for i = 1:size (groupcatValue,2)
%     logicalIndex = groupcat == groupcatValue(i);
%     sub_Table = dataTable(logicalIndex, :);
%     sub_Table.Map = sub_Table.Map + offset(i);
%     boxChart = boxchart(sub_Table.Map,sub_Table.estimate_error,'BoxWidth',offset_step-0.05);
%     hold on
% end
% 
% %colororder(["#FF8000";"#006633"; "#0072BD"])
% lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
% set(lines, 'Color', 'r');
% set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
% xposition = [0,1,2,3,4,5];
% set(gca,'Xtick',xposition);
% xlim([-0.5,5.5])
% grid on;
% yticks(0:2:10);
% ylim([-1,10]);
% xlabel('Map id','FontSize',12);
% title('Estimation error (m)','FontSize',12);
% legend(groupcatValue,'FontSize',11);
% 
% axes(ha(2));
% for i = 1:size (groupcatValue,2)
%     logicalIndex = groupcat == groupcatValue(i);
%     sub_Table = dataTable(logicalIndex, :);
%     sub_Table.Map = sub_Table.Map + offset(i);
%     boxChart = boxchart(sub_Table.Map,sub_Table.iteration,'BoxWidth',offset_step-0.05);
%     hold on
% end
% lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
% set(lines, 'Color', 'r');
% set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
% grid on;
% ylim([0,40])
% xlim([-0.5,5.5])
% xlabel('Map id','FontSize',12)
% title('Iterations','FontSize',12)
% legend(groupcatValue,'FontSize',11)
% 
% saveas(f,[plot_folder '/MOX_new_KLD_stop_Vs_visit_Vs_continuous.png']);
% saveas(f,[plot_folder '/MOX_new_KLD_stop_Vs_visit_Vs_continuous.eps'],'epsc');
%% Compare PID sensor with KLD planning: stop-sense-go vs visit-sense-go

% %the general folder
general_folder_list = ["../Results/ICRA2024/Final_campaign/PID/KLD_visit_sense_PID",...
     "../Results/ICRA2024/Final_campaign/PID/KLD_stop_sense_PID"];
num_comp = size(general_folder_list,2);

% load all the data and combine the results
for i = 1:num_comp
    tbl = readtable(strcat(general_folder_list(i),"/experiment_data.csv"));
    dataTable = [dataTable; tbl];
end

% box plot of comparing the method
f = figure('Position',[-1561         656         1215         615]);
[ha, pos] = tight_subplot(1,2,[.2 .04],[.12 .1],[.03 .01])
%sgtitle("Comparison between stop/visit-sense-go and sample in motion with MOX sensor", 'FontSize',15)
%sgtitle("Comparison between informative kld and belief clustering with continuous MOX sensor", 'FontSize',15)
samplingModeOrder = ["stop sense go", "visit sense go"];
dataTable.sampling_mode = categorical(dataTable.sampling_mode, samplingModeOrder);
planningModeOrder = ["informative kld", "belief clustering"];
dataTable.planning_mode = categorical(dataTable.planning_mode, planningModeOrder);

% change the name of map 5 to 4, and map 10 to map 5 to keep consistency
% Create a logical index for rows where Map is 5
logicalIndex = dataTable.Map == 5;

% Replace all occurrences of 5 with 4 using logical indexing
dataTable.Map(logicalIndex) = 4;

% Create a logical index for rows where Map is 10
logicalIndex = dataTable.Map == 10;

% Replace all occurrences of 10 with 5 using logical indexing
dataTable.Map(logicalIndex) = 5;


groupcat = dataTable.sampling_mode;
groupcatValue = samplingModeOrder;

% we want to keep more distance between different group while put the box
% belong to the same group closer
% Create a logical index for rows where planning_mode is "informative kld"
offset_step = 0.4/(size(groupcatValue,2)-1);
offset= -0.2:offset_step:0.2;
axes(ha(1));
for i = 1:size (groupcatValue,2)
    logicalIndex = groupcat == groupcatValue(i);
    sub_Table = dataTable(logicalIndex, :);
    sub_Table.Map = sub_Table.Map + offset(i);
    boxChart = boxchart(sub_Table.Map,sub_Table.estimate_error,'BoxWidth',offset_step-0.12);
    hold on
end

colororder(["#EDB120", "#77AC30"])
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
xposition = [0,1,2,3,4,5];
set(gca,'Xtick',xposition);
xlim([-0.5,5.5])
grid on;
yticks(0:2:10);
ylim([-1,10]);
xlabel('Map id','FontSize',12);
title('Estimation error (m)','FontSize',12);
legend(groupcatValue,'FontSize',11);

axes(ha(2));
for i = 1:size (groupcatValue,2)
    logicalIndex = groupcat == groupcatValue(i);
    sub_Table = dataTable(logicalIndex, :);
    sub_Table.Map = sub_Table.Map + offset(i);
    boxChart = boxchart(sub_Table.Map,sub_Table.iteration,'BoxWidth',offset_step-0.12);
    hold on
end
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
grid on;
ylim([0,40])
xlim([-0.5,5.5])
xlabel('Map id','FontSize',12)
title('Iterations','FontSize',12)
legend(groupcatValue,'FontSize',11)

saveas(f,[plot_folder '/PID_KLD_stop_Vs_visit.png']);
saveas(f,[plot_folder '/PID_KLD_stop_Vs_visit.eps'],'epsc');

