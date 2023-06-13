%% run t-test between different experiment to determine if there is a statistical difference
general_folder = '/home/wjin/Workspace/continuous_STE/Results/';
experiment_1 = 'results_stopsense_PID';
experiment_2 = 're-evulate_ICRA';
experiment_1_folder = [general_folder experiment_1];
experiment_2_folder = [general_folder experiment_2];

% load error list
statistics_experiment_1 = load([experiment_1_folder '/error_statistics.mat']);
statistics_experiment_2 = load([experiment_2_folder '/error_statistics.mat']);

%% run the t-test for error_x
x_error_exp_1 = statistics_experiment_1.error_x;
x_mean_exp_1 = mean(x_error_exp_1);
x_std_exp_1 = std(x_error_exp_1);

x_error_exp_2 = statistics_experiment_2.error_x;
x_mean_exp_2 = mean(x_error_exp_2 );
x_std_exp_2 = std(x_error_exp_2 );

[h, p, ci, stats] = ttest2(x_error_exp_1, x_error_exp_2);
fprintf('mean_x_1 is: %d, mean_x_2 is:%d \n', x_mean_exp_1, x_mean_exp_2);
fprintf('std_x_1 is: %d, std_x_2 is:%d \n', x_std_exp_1, x_std_exp_2);
fprintf('Results of t-test for error_x:\n');
fprintf('Null Hypothesis (H0) is rejected: %d\n', h);
fprintf('p-value: %.4f\n', p);
fprintf('Confidence Interval: [%.2f, %.2f]\n', ci(1), ci(2));
fprintf('Test Statistic: %.4f\n', stats.tstat);
fprintf('Degrees of Freedom: %d\n', stats.df);
fprintf('----------------------------------------\n');

%% run the t-test for error_y
y_error_exp_1 = statistics_experiment_1.error_y;
y_mean_exp_1 = mean(y_error_exp_1);
y_std_exp_1 = std(y_error_exp_1);

y_error_exp_2 = statistics_experiment_2.error_y;
y_mean_exp_2 = mean(y_error_exp_2 );
y_std_exp_2 = std(y_error_exp_2 );

[h, p, ci, stats] = ttest2(y_error_exp_1, y_error_exp_2);

fprintf('mean_y_1 is: %d, mean_y_2 is:%d \n', y_mean_exp_1, y_mean_exp_2);
fprintf('std_y_1 is: %d, std_y_2 is:%d \n', y_std_exp_1, y_std_exp_2);
fprintf('Results of t-test for error_y:\n');
fprintf('Null Hypothesis (H0) is rejected: %d\n', h);
fprintf('p-value: %.4f\n', p);
fprintf('Confidence Interval: [%.2f, %.2f]\n', ci(1), ci(2));
fprintf('Test Statistic: %.4f\n', stats.tstat);
fprintf('Degrees of Freedom: %d\n', stats.df);
fprintf('----------------------------------------\n');

%% run the t-test for iteration_num
iter_exp_1 = statistics_experiment_1.iteration_nbr_list;
iter_mean_exp_1 = mean(iter_exp_1);
iter_std_exp_1 = std(iter_exp_1);

iter_exp_2 = statistics_experiment_2.iteration_nbr_list;
iter_mean_exp_2 = mean(iter_exp_2);
iter_std_exp_2 = std(iter_exp_2);

[h, p, ci, stats] = ttest2(iter_exp_1, iter_exp_2);

fprintf('mean_iter_1 is: %d, mean_iter_2 is:%d \n', iter_mean_exp_1, iter_mean_exp_2);
fprintf('std_iter_1 is: %d, std_iter_2 is:%d \n', iter_std_exp_1, iter_std_exp_2);
fprintf('Results of t-test for iteration:\n');
fprintf('Null Hypothesis (H0) is rejected: %d\n', h);
fprintf('p-value: %.4f\n', p);
fprintf('Confidence Interval: [%.2f, %.2f]\n', ci(1), ci(2));
fprintf('Test Statistic: %.4f\n', stats.tstat);
fprintf('Degrees of Freedom: %d\n', stats.df);