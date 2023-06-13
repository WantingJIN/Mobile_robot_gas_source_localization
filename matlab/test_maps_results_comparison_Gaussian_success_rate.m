clear all; clc; close all;

%% GET FIRST DATASET
% the general folder
general_folder = ["/home/wjin/Workspace/continuous_STE/Results/results_test_maps_4000_dataset"
                   "/home/wjin/Workspace/continuous_STE/Results/results_test_maps_Gaussian_5D"]  ;
save_folder_path = '/home/wjin/Workspace/continuous_STE/Results';
n_exp=10;
               
for gf=1:size(general_folder,1)
    
    folder_list = dir(general_folder(gf));
    folder_list = folder_list(~ismember({folder_list.name},{'.','..','current','101','102','103'}));
    folder_list = folder_list([folder_list.isdir]);
    exp_number = length(folder_list);
    
    
    for i=1:exp_number
        % the name of the folder
        folder_name = [folder_list(i).folder, '/', folder_list(i).name];
        % the content of the folder
        robot_list = dir(folder_name);
        robot_list = robot_list(~ismember({robot_list.name},{'.','..'}));
        robot_list = robot_list([robot_list.isdir]);
        % the IDs of the robots
        ID = arrayfun(@(struct) str2double(struct.name),robot_list);

        % skip if it's not a job folder
        if(~exist([folder_name, '/', num2str(ID(1)), '/param_log.csv'],'file')), continue; end

        % get simulation's parameter values
        params = my_text2struct([folder_name, '/', num2str(ID(1)), '/param_log.csv']);

        % remove folders with unfinished jobs
        file_list = dir([folder_name '/**']);
        if(sum(ismember({file_list.name},{'position_history.csv'})) == 0), rmdir(folder_name,'s'); continue; end
        entropy = dlmread([folder_name, '/', num2str(ID(1)), '/entropy.csv']);
        if(entropy(end,1) > 0.1 && size(entropy,1) < params.max_iter_number), rmdir(folder_name,'s'); continue; end

        % the right path of the folder
        new_path = [general_folder(gf) '/' num2str(params.map_nbr)];

        % move the folder with its friends in the right path
        if(~exist(new_path,'dir'))
            mkdir(new_path);
        end
        movefile(folder_name, new_path);
    end
%%

    % choose the folders
    new_dir=general_folder(gf)+"/**";
    folder_list = dir([new_dir]);
    folder_list = folder_list(~ismember({folder_list.name},{'.','..', '101', '102','103'}));
    folder_list = folder_list(~ismember({folder_list.folder}, general_folder(gf)));
    folder_list = folder_list([folder_list.isdir]);
    number = length(folder_list);

    %% get data
    for i=1:number
        folder_name = [folder_list(i).folder, '/', folder_list(i).name];
        % multiple robots
        robot_list = dir(folder_name);
        robot_list = robot_list(~ismember({robot_list.name},{'.','..'}));
        robot_list = robot_list([robot_list.isdir]);
        ID = arrayfun(@(struct) str2num(struct.name),robot_list);
        % remove folders with unfinished jobs
        file_list = dir([folder_name '/**']);
        if(sum(ismember({file_list.name},{'position_history.csv'})) == 0), rmdir(folder_name,'s'); continue; end
        entropy = dlmread([folder_name, '/', num2str(ID(1)), '/entropy.csv']);
        % ground truth and other params
        params = my_text2struct([folder_name, '/', num2str(ID(1)), '/param_log.csv']);
        map_nbr_list(i,gf) = params.map_nbr;
        % stats
        expected_a_posteriori_file = dlmread([folder_list(i).folder, '/', folder_list(i).name, '/' num2str(ID(1)), '/expected_a_posteriori.csv']);
        iteration_nbr_list(gf,i) = size(expected_a_posteriori_file,1);
        expected_a_posteriori(i,1:2) = expected_a_posteriori_file(end,2:3);
        % error on X and Y (col 5 and 6)
        max_a_post = dlmread([folder_list(i).folder, '/', folder_list(i).name, '/' num2str(ID(1)), '/max_a_posteriori.csv']);
        if(isnan(expected_a_posteriori(i,1))), expected_a_posteriori(i,1) = max_a_post(end,2); end
        if(isnan(expected_a_posteriori(i,2))), expected_a_posteriori(i,2) = max_a_post(end,3); end
        error_list(gf,i,1) = params.source_position(1) - expected_a_posteriori(i,1);
        error_list(gf,i,2) = params.source_position(2) - expected_a_posteriori(i,2);
        entropy = dlmread([folder_list(i).folder, '/', folder_list(i).name, '/' num2str(ID(1)), '/entropy.csv']);
        entropy_list(i,1) = entropy(end,1);

        max_a_post_list(i,1:2) = max_a_post(end,2:3);
        convergence_list(i,1) = sqrt((max_a_post_list(i,1) - expected_a_posteriori(i,1))^2 + (max_a_post_list(i,2) - expected_a_posteriori(i,2))^2);
    end
    
end
%%Get success rate and put it in pie chart
pie_data=zeros(size(general_folder,1),n_exp,2);
for gf=1:size(general_folder,1)
    success_x=(1*(error_list(gf,:,1)>1)+1*(error_list(gf,:,1)<-1)+ 1*(error_list(gf,:,2)<-0.5) + 1*(error_list(gf,:,2)>0.5))';
    for i=1:n_exp
        pie_data(gf,i,:)=[sum(success_x(map_nbr_list(:,gf)==(i),1)==0) sum(success_x(map_nbr_list(:,gf)==(i),1) ~= 0)];
    end
end

%Some constants for plots
piechar_textsize=0.001;
piechart_y_offset=7;
piechart_shrink_factor=0.2;

%Plot positioning
g1 = repmat({'0.1'},n_exp,1);
g2 = repmat({'0.2'},n_exp,1);
g3 = repmat({'0.3'},n_exp,1);
g4 = repmat({'0.4'},n_exp,1);
g5 = repmat({'0.5'},n_exp,1);
g6 = repmat({'0.6'},n_exp,1);
g7 = repmat({'0.7'},n_exp,1);
g8 = repmat({'0.8'},n_exp,1);
g1_2 = repmat({'A'},n_exp,1);
g2_2 = repmat({'B'},n_exp,1);
g3_2 = repmat({'C'},n_exp,1);
g4_2 = repmat({'D'},n_exp,1);
g5_2 = repmat({'E'},n_exp,1);
g6_2 = repmat({'F'},n_exp,1);
g7_2 = repmat({'G'},n_exp,1);
g8_2 = repmat({'H'},n_exp,1);
g = [g1; g1_2; g2; g2_2; g3; g3_2; g4; g4_2; g5; g5_2; g6; g6_2; g7; g7_2; g8; g8_2];
positions2plot= [1 1.2   1.6 1.8   2.2 2.4   2.8 3   3.4 3.6   4.0 4.2   4.6 4.8   5.2 5.4];

f = figure();
f.Position = [100 100 540 550];
h=plot([0 length(unique(map_nbr_list))], [0 0], 'g');
hold on;
x=[error_list(1,1:10,1)';error_list(2,1:10,1)';error_list(1,11:20,1)';error_list(2,11:20,1)';error_list(1,21:30,1)';error_list(2,21:30,1)';error_list(1,31:40,1)';error_list(2,31:40,1)';error_list(1,41:50,1)';error_list(2,41:50,1)';error_list(1,51:60,1)';error_list(2,51:60,1)';error_list(1,61:70,1)';error_list(2,61:70,1)';error_list(1,71:80,1)';error_list(2,71:80,1)'];
color2 = [241 197 45]./255;
color1 = [44,136,217]./255;
custom_color = [color1;color2;color1;color2;color1;color2;color1;color2;color1;color2;color1;color2;color1;color2;color1;color2];
%boxplot(x,g,'Colors','brbrbrbrbrbrbrbr','positions', positions2plot);
boxplot(x,g,'Colors',custom_color,'positions', positions2plot,'Widths',0.15);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(gca,'xtick',[mean(positions2plot(1:2)) mean(positions2plot(3:4)) mean(positions2plot(5:6)) mean(positions2plot(7:8)) mean(positions2plot(9:10)) mean(positions2plot(11:12)) mean(positions2plot(13:14)) mean(positions2plot(15:16))]);
set(gca,'xticklabel',{'1','2', '3', '4', '5','6','7','8'},'fontsize',10);

%boxplot(error_list(:,1), map_nbr_list);

        
           colormap1=([.7 .7 .7;   %color for successes
                    1 1 1]);  %color for failures
      
           colormap2=([.1 .2 .2;   %color for successes
                    1 1 1]);
       
for gf=1:size(general_folder,1)
    for i=1:length(unique(map_nbr_list(:,gf)))
        p=pie(pie_data(gf,i,:));
        %Change colors here
        colormap([.1 .1 .1;   %color for successes
                    1 1 1]);  %color for failures

        if gf==1
            pointer=1;
        else
            pointer=0;
        end
        for k=1:length(p)
            if strcmp(get(p(k),'Type'),'patch') % Patch graphics
                  XData = get(p(k),'XData');  % Extract current data
                  YData = get(p(k),'YData');
                  
                  set(p(k),'XData',XData*piechart_shrink_factor*0.4 +positions2plot(2*i-pointer)); % Insert modified data
                  set(p(k),'YData',YData*piechart_shrink_factor +piechart_y_offset);
            else
                 Pos = get(p(k),'Position'); % Get the pie chart text
                 set(p(k),'Position',[(Pos + [i piechart_y_offset 1]).*[1 1 piechar_textsize]])
                 hidem(p(k)); % hide the pie chart text 
            end
        end
    end
end

text(mean(positions2plot(6:7)),8,'Success Rate','fontweight','bold','fontsize',15); %additional text
grid on;
ylim([-1,10])
xlabel('Map number');
ylabel('Error(m)');
set(findobj(gca,'type','line'),'linew',2)
boxes = findobj(gca, 'Tag', 'Box');
legend(boxes([end 1]), 'DDPM', 'Pseudo-Gaussian','fontsize',12,'Location','northeast')
title('Error on x','FontSize', 15);
%saveas(f,[save_folder_path '/error_x.eps']);
%%
f = figure();
f.Position = [100 100 540 550];
h=plot([0 length(unique(map_nbr_list))], [0 0], 'g');
hold on;
x=[error_list(1,1:10,2)';error_list(2,1:10,2)';error_list(1,11:20,2)';error_list(2,11:20,2)';error_list(1,21:30,2)';error_list(2,21:30,2)';error_list(1,31:40,2)';error_list(2,31:40,2)';error_list(1,41:50,2)';error_list(2,41:50,2)';error_list(1,51:60,2)';error_list(2,51:60,2)';error_list(1,61:70,2)';error_list(2,61:70,2)';error_list(1,71:80,2)';error_list(2,71:80,2)'];
color2 = [241 197 45]./255;
color1 = [44,136,217]./255;
custom_color = [color1;color2;color1;color2;color1;color2;color1;color2;color1;color2;color1;color2;color1;color2;color1;color2];
%boxplot(x,g,'Colors','brbrbrbrbrbrbrbr','positions', positions2plot);
boxplot(x,g,'Colors',custom_color,'positions', positions2plot,'Widths',0.15);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
%set(lines(1:2:end), 'Color', 'r');
%set(lines(2:2:end), 'Color', 'g');
set(gca,'xtick',[mean(positions2plot(1:2)) mean(positions2plot(3:4)) mean(positions2plot(5:6)) mean(positions2plot(7:8)) mean(positions2plot(9:10)) mean(positions2plot(11:12)) mean(positions2plot(13:14)) mean(positions2plot(15:16))]);
set(gca,'xticklabel',{'1','2', '3', '4', '5','6','7','8'},'fontsize',10);
grid on;
ylim([-2,1.5])
xlabel('Map number');
ylabel('Error(m)');
set(findobj(gca,'type','line'),'linew',2)
boxes = findobj(gca, 'Tag', 'Box');
legend(boxes([end 1]), 'DDPM', 'Pseudo-Gaussian','fontsize',12,'Location','northeast')
outliers = findobj(gca,'Tag','Outliers');
set(outliers(2:2:end), 'Color', 'g');
set(outliers(2:2:end), 'Color', 'g');
title('Error on y','FontSize', 15);
%saveas(f,[save_folder_path '/error_y.eps']);
%%
f = figure();
f.Position = [100 100 540 550];
h=plot([0 length(unique(map_nbr_list))], [0 0], 'g');
hold on;
x=[iteration_nbr_list(1,1:10)';iteration_nbr_list(2,1:10)';iteration_nbr_list(1,11:20)';iteration_nbr_list(2,11:20)';iteration_nbr_list(1,21:30)';iteration_nbr_list(2,21:30)';iteration_nbr_list(1,31:40)';iteration_nbr_list(2,31:40)';iteration_nbr_list(1,41:50)';iteration_nbr_list(2,41:50)';iteration_nbr_list(1,51:60)';iteration_nbr_list(2,51:60)';iteration_nbr_list(1,61:70)';iteration_nbr_list(2,61:70)';iteration_nbr_list(1,71:80)';iteration_nbr_list(2,71:80)'];
color2 = [241 197 45]./255;
color1 = [44,136,217]./255;
custom_color = [color1;color2;color1;color2;color1;color2;color1;color2;color1;color2;color1;color2;color1;color2;color1;color2];
%boxplot(x,g,'Colors','brbrbrbrbrbrbrbr','positions', positions2plot);
boxplot(x,g,'Colors',custom_color,'positions', positions2plot,'Widths',0.15);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
% set(lines(1:2:end), 'Color', 'r');
% set(lines(2:2:end), 'Color', 'g');
%outliers = findobj(gcf,'type', 'marker','Tag','Outliers');
% set(outliers(1:2:end), 'Color', 'g');
% set(outliers(2:2:end), 'Color', 'g');
set(gca,'xtick',[mean(positions2plot(1:2)) mean(positions2plot(3:4)) mean(positions2plot(5:6)) mean(positions2plot(7:8)) mean(positions2plot(9:10)) mean(positions2plot(11:12)) mean(positions2plot(13:14)) mean(positions2plot(15:16))]);
set(gca,'xticklabel',{'1','2', '3', '4', '5','6','7','8'},'fontsize',10);
grid on;
%ylim([-2,2])
xlabel('Map number');
set(findobj(gca,'type','line'),'linew',2)
boxes = findobj(gca, 'Tag', 'Box');
legend(boxes([end 1]), 'DDPM', 'Pseudo-Gaussian','fontsize',12,'Location','northeast')
title('Iteration numbers','FontSize', 15);
%saveas(f,[save_folder_path '/iteration.eps']);
%boxplot(error_list(:,1), map_nbr_list);

%% boxplots
% f = figure('units','normalized','outerposition',[0 0 1 1]);
% subplot(5,1,1)
% plot([0 length(unique(map_nbr_list))], [0 0], 'g');
% hold on;
% boxplot(error_list(:,1), map_nbr_list);
% grid on;
% title('error on x');
% subplot(5,1,2);
% plot([0 length(unique(map_nbr_list))], [0 0], 'g');
% hold on;
% boxplot(error_list(:,2), map_nbr_list);
% grid on;
% title('error on y');
% subplot(5,1,3);
% boxplot(iteration_nbr_list, map_nbr_list);
% grid on;
% title('nbr iterations');
% subplot(5,1,4);
% boxplot(entropy_list, map_nbr_list);
% grid on;
% title('final entropy');
% subplot(5,1,5);
% boxplot(convergence_list, map_nbr_list);
% grid on;
% title('convergence');
% 
% %% save
% saveas(f,[general_folder '/test_maps_results_comparison_all.png']);


%% plot for thesis
% f = figure();
% 
% subplot(1,3,1)
% plot([0 length(unique(map_nbr_list))], [0 0], 'g');
% hold on;
% boxplot(error_list(:,1), map_nbr_list);
% grid on;
% ylim([-1,10])
% xlabel('Map number');
% title('Error on x');
% subplot(1,3,2);
% plot([0 length(unique(map_nbr_list))], [0 0], 'g');
% hold on;
% boxplot(error_list(:,2), map_nbr_list);
% grid on;
% ylim([-3,4])
% xlabel('Map number');
% title('Error on y');
% subplot(1,3,3);
% boxplot(iteration_nbr_list, map_nbr_list);
% grid on;
% ylim([0,50])
% xlabel('Map number');
% title('nbr iterations');


%saveas(f,[general_folder '/test_maps_results_comparison.png']);