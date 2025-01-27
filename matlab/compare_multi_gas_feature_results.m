

%% Plot result for the robustness towards different release rate
% Create an empty table to store experimental data
clear all; clc; close all;
dataTable = table();

plot_folder='../Results/RAL_2025_results/Plot';
% load the data 
% the general folder
general_folder_list = ["../Results/RAL_2025_results/simu_results/release_rate_robustness/absolute_meas",...
    "../Results/RAL_2025_results/simu_results/release_rate_robustness/empirical_cdf", ...
    "../Results/RAL_2025_results/simu_results/release_rate_robustness/gas_hit_fixed_thres_num", ...
    "../Results/RAL_2025_results/simu_results/release_rate_robustness/gas_hit_sliding_window_num"];
% general_folder_list = ["/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/MOX/KLD_continunous_MOX",...
%     "/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/MOX/clustering_MOX_continuous_new_cluster_selection"];
%  general_folder_list = ["/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/PID/KLD_visit_sense_PID",...
%      "/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/PID/KLD_stop_sense_PID"];
num_comp = size(general_folder_list,2);

% load all the data and combine the results
for i = 1:num_comp
    tbl = readtable(strcat(general_folder_list(i),"/experiment_data.csv"));
    dataTable = [dataTable; tbl];
end

% box plot of comparing the method
% f = figure('Position',[-1561         656         1215         615]);
f = figure('Position',[-1561         656         575         355]);
[ha, pos] = tight_subplot(1,2,[.2 .06],[.13 .1],[.04 .01])
%sgtitle("Comparison between stop/visit-sense-go and sample in motion with MOX sensor", 'FontSize',15)
%sgtitle("Comparison between informative kld and belief clustering with continuous MOX sensor", 'FontSize',15)
%samplingModeOrder = ["stop sense go", "visit sense go"];
gasFeatureOrder = ["gas concentration", "fixed gas hit", "adaptive gas hit", "gas measurement rank"];

dataTable.gas_feature = categorical(dataTable.gas_feature, gasFeatureOrder);
groupcat = dataTable.gas_feature;
groupcatValue = gasFeatureOrder;


releaseRateOrder=[10, 53, 200];
dataTable.release_rate = categorical(dataTable.release_rate, releaseRateOrder);
%subplot(1,3,1)
axes(ha(1));
boxChart = boxchart(dataTable.release_rate,dataTable.estimate_error,'GroupByColor',groupcat,'BoxWidth',0.6);
% we want to keep more distance between different group while put the box
% belong to the same group closer
% Create a logical index for rows where planning_mode is "informative kld"
% offset_step = 0.4/(size(groupcatValue,2)-1);
% offset= -0.2:offset_step:0.2;
% for i = size (groupcatValue)
%     logicalIndex = groupcat == groupcatValue(i);
%     sub_Table = dataTable(logicalIndex, :);
%     release_rate_order_numeric = double(sub_Table.release_rate_order);
%     release_rate_with_offset = release_rate_order_numeric + offset(i);
%     boxChart = boxchart(release_rate_with_offset,sub_Table.estimate_error,'BoxWidth',offset_step);
%     hold on
% end

%colororder(["#EDB120", "#77AC30"])
% colororder(["#89ABE3"; "#EA738D"])
colororder(["#FF8000";"#006633"; "#0072BD"; "#EA738D"])
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
set(gca,'XTickLabel',{'low', 'default', 'high'});
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
yticks(0:2:10);
ylim([-1,10])
xlabel('Release rate','FontSize',12)
title('Estimation error (m)','FontSize',12)
% legend('FontSize',12)
%subplot(1,3,2)
axes(ha(2));
boxchart(dataTable.release_rate,dataTable.iteration,'GroupByColor',groupcat,'BoxWidth',0.6)
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
set(gca,'XTickLabel',{'low', 'default', 'high'});
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([0,35])
xlabel('Release rate','FontSize',12)
title('Iterations','FontSize',12)
legend('FontSize',12)
saveas(f,[plot_folder '/robustness_release_rate.png']);
saveas(f,[plot_folder '/robustness_release_rate.eps'],'epsc');
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

%% plot the results towards different sensor model
% load the data for the robustness towards different release rate
% the general folder
% Create an empty table to store experimental data
clear all; clc; close all;
dataTable = table();
plot_folder='../Results/paper_results/Plot';
general_folder_list = ["../Results/RAL_2025_results/simu_results/sensor_model_robustness/absolute_meas",...
    "../Results/RAL_2025_results/simu_results/sensor_model_robustness/empirical_cdf", ...
    "../Results/RAL_2025_results/simu_results/sensor_model_robustness/gas_hit_fixed_thres_num", ...
    "../Results/RAL_2025_results/simu_results/sensor_model_robustness/gas_hit_sliding_window_num"];
% general_folder_list = ["/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/MOX/KLD_continunous_MOX",...
%     "/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/MOX/clustering_MOX_continuous_new_cluster_selection"];
%  general_folder_list = ["/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/PID/KLD_visit_sense_PID",...
%      "/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/PID/KLD_stop_sense_PID"];
num_comp = size(general_folder_list,2);

% load all the data and combine the results
for i = 1:num_comp
    tbl = readtable(strcat(general_folder_list(i),"/experiment_data.csv"));
    dataTable = [dataTable; tbl];
end

% box plot of comparing the method
% f = figure('Position',[-1561         656         1215         615]);
f = figure('Position',[-1561         656        575         355]);
[ha, pos] = tight_subplot(1,2,[.2 .06],[.13 .1],[.04 .01])
%sgtitle("Comparison between stop/visit-sense-go and sample in motion with MOX sensor", 'FontSize',15)
%sgtitle("Comparison between informative kld and belief clustering with continuous MOX sensor", 'FontSize',15)
%samplingModeOrder = ["stop sense go", "visit sense go"];
gasFeatureOrder = ["gas concentration", "fixed gas hit", "adaptive gas hit", "gas measurement rank"];

dataTable.gas_feature = categorical(dataTable.gas_feature, gasFeatureOrder);
groupcat = dataTable.gas_feature;

R0Order=[100 1500];
dataTable.R0 = categorical(dataTable.R0, R0Order);
%subplot(1,3,1)
axes(ha(1));
boxChart = boxchart(dataTable.R0,dataTable.estimate_error,'GroupByColor',groupcat,'BoxWidth',0.6);
%colororder(["#EDB120", "#77AC30"])
% colororder(["#89ABE3"; "#EA738D"])
colororder(["#FF8000";"#006633"; "#0072BD"; "#EA738D"])
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
set(gca,'XTickLabel',{'100', '1500'});
grid on;
yticks(0:2:10);
ylim([-1,10])
xlabel('R0 [k\Omega]','FontSize',12)
title('Estimation error (m)','FontSize',12)
% legend('FontSize',12)
%subplot(1,3,2)
axes(ha(2));
boxchart(dataTable.R0,dataTable.iteration,'GroupByColor',groupcat,'BoxWidth',0.6)
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
% set(gca,'XTickLabel',{'0', '1', '2' , '3', '4', '5'});
set(gca,'XTickLabel',{'100', '1500'});
grid on;
ylim([0,35])
xlabel('R0 [k\Omega]','FontSize',12)
title('Iterations','FontSize',12)
legend('FontSize',12)
saveas(f,[plot_folder '/robustness_sensor_model.png']);
saveas(f,[plot_folder '/robustness_sensor_model.eps'],'epsc');
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

