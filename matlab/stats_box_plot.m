clear all; clc; close all;
% general_folder = '/data/FaezehRahbar/Projects/OSL_STE_multi-robot/Results/webots';

for a=1
%     if(a==1), folder_list = dir([general_folder '/0_no_com_no_coor/1_robot']); end
%     if(a==2), folder_list = dir([general_folder '/0_no_com_no_coor/2_robots']); end
%     if(a==3), folder_list = dir([general_folder '/0_no_com_no_coor/3_robots']); end
%     
%     if(a==4), folder_list = dir([general_folder '/1_com_no_coor/2_robots']); end
%     if(a==5), folder_list = dir([general_folder '/1_com_no_coor/3_robots']); end
%     if(a==8), folder_list = dir('/data/FaezehRahbar/Projects/OSL_STE_multi-robot/Results/1_com_no_coor/3_robots_2'); end
%     
%     if(a==6), folder_list = dir([general_folder '/2_com_coor/2_robots']); end
%     if(a==7), folder_list = dir([general_folder '/2_com_coor/3_robots']); end
%     if(a==9), folder_list = dir('/data/FaezehRahbar/Projects/OSL_STE_multi-robot/Results/2_com_coor/3_robots_2'); end
%     
%     if(a==10), folder_list = dir('/data/FaezehRahbar/Projects/OSL_STE_multi-robot/Results/friends_goal_prediction/2_robots'); end
%     if(a==11), folder_list = dir('/data/FaezehRahbar/Projects/OSL_STE_multi-robot/Results/friends_goal_prediction/3_robots'); end
%     
%     if(a==12), folder_list = dir('/data/FaezehRahbar/Projects/OSL_STE_multi-robot/Results/2nd_round/2/2_robots'); end
%     if(a==13), folder_list = dir('/data/FaezehRahbar/Projects/OSL_STE_multi-robot/Results/2nd_round/2/3_robots'); end
%     
%     if(a==0), folder_list = dir('/data/FaezehRahbar/Projects/OSL_STE_multi-robot/Results/new_braitenberg_internal_timeout'); end
    
    % choose the folders
    folder_list = dir('/home/wjin/Workspace/continuous_STE/Results/results_test_maps_4000_dataset/2');
    folder_list = folder_list(~ismember({folder_list.name},{'.','..','current'}));
    folder_list = folder_list([folder_list.isdir]);
    exp_number = length(folder_list);
    
    % empty arrays
    expected_list = zeros(exp_number, 5);
    max_list = zeros(exp_number, 5);
    num_iteration = zeros(exp_number,1);
    entropy_list = zeros(exp_number,1);
    travelled_dist = zeros(exp_number,1);
    spent_time = zeros(exp_number,1);
    
    
    %% gather stats
    for i=1:exp_number
        folder_name = [folder_list(end-i+1).folder, '/', folder_list(end-i+1).name];
        % multiple robots
        robot_list = dir(folder_name);
        robot_list = robot_list(~ismember({robot_list.name},{'.','..'}));
        robot_list = robot_list([robot_list.isdir]);
        ID = arrayfun(@(struct) str2num(struct.name),robot_list);
        % ground truth and other params
        params = my_text2struct([[folder_name, '/', num2str(ID(1))], '/param_log.csv']);
        source_real_position = params.source_position;
        Q_min = params.min_maxs(1); Q_max = params.min_maxs(2);
        source_position_min = [params.min_maxs(3),params.min_maxs(5)];
        source_position_max = [params.min_maxs(4),params.min_maxs(6)];
        a_y_min = params.min_maxs(7); a_y_max = params.min_maxs(8);
        b_y_min = params.min_maxs(9); b_y_max = params.min_maxs(10);
        Q_nbins = params.nbins(1);
        x_nbins = params.nbins(2);
        y_nbins = params.nbins(3);
        a_y_nbins = params.nbins(4);
        b_y_nbins = params.nbins(5);
        finish_time = 10e+10; % just an initialization
        for j=1:params.number_of_robots
            folder_name_ID = [folder_name, '/', num2str(ID(j))];
            % data files :
            %position_history = dlmread([folder_name_ID, '/goal_position.csv']); %position_history
            expected_a_posteriori = dlmread([folder_name_ID '/expected_a_posteriori.csv']);
            max_a_posteriori = dlmread([folder_name_ID '/max_a_posteriori.csv']);
            entropy = dlmread([folder_name_ID '/entropy.csv']);
            % data extraction from the robot who finished first:
            if(entropy(end,2) < finish_time)
                finish_time = entropy(end,2);
                spent_time(i) = finish_time;%/1000;
                %expected_list(i,:) = expected_a_posteriori(end,2:end-2);
                expected_list(i,:) = expected_a_posteriori(end,2:end) - [source_real_position(1) source_real_position(2) 0 0 0];
                if(isnan(expected_list(i,1))) 
                    expected_list(i,:) = expected_a_posteriori(end-1,2:end) - [source_real_position(1) source_real_position(2) 0 0 0];
                end
                %max_list(i,:) = max_a_posteriori(end,3:end-2);
                max_list(i,:) = max_a_posteriori(end,2:end) - [source_real_position(1) source_real_position(2) 0 0 0];
                entropy_list(i) = entropy(end,1);
            end
            %         num_iteration(i) = num_iteration(i) + sum(entropy(:,2) < finish_time);
            %         travelled_dist(i) = travelled_dist(i) + travelled_distance_calculation(position_history,1,sum(entropy(:,2) < finish_time));
        end
        for j=1:params.number_of_robots
            folder_name_ID = [folder_name, '/', num2str(ID(j))];
            position_history = dlmread([folder_name_ID, '/goal_position.csv']);
            num_iteration(i) = num_iteration(i) + sum(position_history(:,5) < finish_time);
            travelled_dist(i) = travelled_dist(i) + travelled_distance_calculation(position_history,1,sum(position_history(:,5) < finish_time));
            if(travelled_dist(i) > 2e4)
                disp(folder_name_ID);
                params.number_of_robots
            end
        end
%         if(entropy_list(i) > 0.05), rmdir(folder_name, 's'); end
    end
    %%
%     bad_runs = entropy_list>0.1;
%     travelled_dist(bad_runs)=[];
%     spent_time(bad_runs)=[];
%     num_iteration(bad_runs)=[];
%     expected_list(bad_runs,:)=[];
%     max_list(bad_runs,:)=[];
%     entropy_list(bad_runs,:)=[];
    
    %% plot
    f0 = figure('Position', [-1919          28         753        1066]);
    
    if(size(expected_list,2)==6)
        
        subplot(3,2,1);
        plot([0,3], [source_real_position(1), source_real_position(1)], 'g');
        hold on;
        boxplot([expected_list(:,2), max_list(:,2)]);
        ylim([source_position_min(1) source_position_max(1)]);
        xticklabels({'expected','max'})
        grid on;
        title('X');
        
        subplot(3,2,3);
        plot([0,3], [source_real_position(2), source_real_position(2)], 'g');
        hold on;
        boxplot([expected_list(:,3), max_list(:,3)]);
        ylim([source_position_min(2) source_position_max(2)]);
        xticklabels({'expected','max'})
        grid on;
        title('Y');
        
        subplot(3,2,5);
        plot([0,3], [source_real_position(3), source_real_position(3)], 'g');
        hold on;
        boxplot([expected_list(:,4), max_list(:,4)]);
        ylim([source_position_min(3) source_position_max(3)]);
        xticklabels({'expected','max'})
        grid on;
        title('Z');
        
        subplot(3,2,2);
        boxplot([expected_list(:,1), max_list(:,1)]);
        ylim([Q_min Q_max]);
        xticklabels({'expected','max'})
        grid on;
        title('Q');
        
        subplot(3,2,4);
        boxplot([expected_list(:,5), max_list(:,5)]);
        ylim([a_y_min a_y_max]);
        xticklabels({'expected','max'})
        grid on;
        title('Sigma_a');
        
        subplot(3,2,6);
        boxplot([expected_list(:,6), max_list(:,6)]);
        ylim([b_y_min b_y_max]);
        xticklabels({'expected','max'})
        grid on;
        title('Sigma_b');
        
    end
    if(size(expected_list,2) == 5)
        
        subplot(3,2,1);
        plot([0,3], [0 0], 'g');
        hold on;
        boxplot([expected_list(:,1), max_list(:,1)]);
        %     ylim([source_position_min(1) source_position_max(1)]);
        xticklabels({'expected','max'})
        grid on;
        title('error on X');
        
        subplot(3,2,3);
        plot([0,3], [0 0], 'g');
        hold on;
        boxplot([expected_list(:,2), max_list(:,2)]);
        %     ylim([source_position_min(2) source_position_max(2)]);
        xticklabels({'expected','max'})
        grid on;
        title('error on Y');
        
        subplot(3,2,2);
        boxplot([expected_list(:,3), max_list(:,3)]);
        ylim([Q_min Q_max]);
        xticklabels({'expected','max'})
        grid on;
        title('Q');
        
        subplot(3,2,4);
        boxplot([expected_list(:,4), max_list(:,4)]);
        ylim([a_y_min a_y_max]);
        xticklabels({'expected','max'})
        grid on;
        title('Sigma_a');
        
        subplot(3,2,6);
        boxplot([expected_list(:,5), max_list(:,5)]);
        ylim([b_y_min b_y_max]);
        xticklabels({'expected','max'})
        grid on;
        title('Sigma_b');
        
    end
    
    % iterations
    f1 = figure('Position', [-1161         674         580         420]);
    boxplot(num_iteration);
    grid on;
    title('number of iterations');
    
    pause(1);
    
    % entropy
    f2 = figure('Position', [-589   680   590   414]);
    boxplot(entropy_list);
    grid on;
    title('final entropy values');
    
    pause(1);
    
    % travelled distance
    f3 = figure('Position', [-1163 77 590 414]);
    boxplot(travelled_dist);
    grid on;
    title('sum of travelled distances');
    
    pause(1);
    
    % spent time
    f4 = figure('Position', [-599    72   600   416]);
    boxplot(spent_time);
    grid on;
    title('experiment duration');
    
    
    %% save
    saveas(f0,[folder_list(1).folder '/max_and_expected_values.png']);
    saveas(f1,[folder_list(1).folder '/number_iterations.png']);
    saveas(f2,[folder_list(1).folder '/final_entropy_values.png']);
    saveas(f3,[folder_list(1).folder '/sum_travelled_dist.png']);
    saveas(f4,[folder_list(1).folder '/sum_exp_duration.png']);
    data = [num_iteration, expected_list, travelled_dist, spent_time];
    save([folder_list(1).folder '/data.mat'],'data');
    
    %%
end

function dist = travelled_distance_calculation(position_history,start_ind,end_ind)
x = position_history(:,2);
y = position_history(:,3);
dist_2pt = sqrt( (x(start_ind+1:end_ind) - x(start_ind:end_ind-1)).^2 + (y(start_ind+1:end_ind) - y(start_ind:end_ind-1)).^2 );
dist = sum(dist_2pt);
end