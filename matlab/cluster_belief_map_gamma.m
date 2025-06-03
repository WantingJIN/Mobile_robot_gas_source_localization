% === Base experiment path (we assume only one subfolder) ===
base_path = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/cluster_analysis';
d = dir(base_path);
subfolder = d([d.isdir] & ~startsWith({d.name}, '.'));  % filter out hidden/system folders

if isempty(subfolder)
    error('No subfolder found inside cluster_analysis');
end

exp_folder = fullfile(base_path, subfolder(1).name);  % full path to experiment folder

% === Visualization speed settings ===
show_cell_indices = false;
repr_vel = 3;           % How many steps to skip per frame (for robot position update)
video_repr_vel = repr_vel*10;    % Frame rate of the output video

% === Robot IDs and corresponding colors ===
robot_ids = [101, 102, 103];
robot_colors = {[1 0 0], [0 0 1], [0 0.6 0]}; % red, blue, green

% === Load position history for each robot ===
positions = cell(1,3);
for r = 1:3
    pos_path = fullfile(exp_folder, num2str(robot_ids(r)), 'whole_position_history.csv');
    pos_data = readmatrix(pos_path);
    positions{r} = round(pos_data(:, 1:2), 1);  % round X, Y positions to 1 decimal
end

% === Load belief map (only from robot 101) ===
belief_path = fullfile(exp_folder, '101', 'posterior_pdf_3D_cluster.csv');
belief_data = readmatrix(belief_path);
belief_x = round(belief_data(:, 1), 1);
belief_y = round(belief_data(:, 2), 1);
belief_maps = belief_data(:, 3:end);  % columns containing map values

% === Load intensity map (direct, no need to skip columns) ===
intensity_path = fullfile(exp_folder, '101', 'posterior_pdf_3D.csv');
intensity_data = readmatrix(intensity_path);
intensity_maps = intensity_data;  % todos los valores son intensidades


% === Load goal positions for each robot ===
goal_positions = cell(1,3);
goal_idx = ones(1,3);  % keep track of which goal each robot is targeting
reached_goals = zeros(1,3); 

goal_update_count = 0;
for r = 1:3
    goal_path = fullfile(exp_folder, num2str(robot_ids(r)), 'goal_position.csv');
    goal_data = readmatrix(goal_path);
    goal_positions{r} = round(goal_data(:, 2:3), 1);  % X, Y columns
end

% === Load ground truth gas source position ===
source_path = fullfile(exp_folder, '101','source_pos.csv');
source_text = fileread(source_path);
tokens = sscanf(source_text, 'source_position %f %f %f');
source_x = tokens(1);
source_y = tokens(2);

% === Setup figure ===
figure('Position', [100, 100, 1200, 400]);
axis equal;
hold on;
wall_thickness = 0.1;
axis([-wall_thickness 16 + wall_thickness -wall_thickness 4 + wall_thickness]);
xlabel('X (m)');
ylabel('Y (m)');
title('FSM cycle: 1');

% Add key press callback for pausing
set(gcf, 'KeyPressFcn', @keypress_callback);
global is_paused;
is_paused = false;

% === Cell grid setup (20x20 grid) with adjusted physical coordinates ===
cols = 20; rows = 20;
x_edges = linspace(3, 12.5, cols + 1);  % 21 edges → 20 cells
y_edges = linspace(0.16, 3.8, rows + 1);
cell_widths = diff(x_edges);   % width of each cell (uniform)
cell_heights = diff(y_edges);  % height of each cell (uniform)

% === Draw grid cells ===
cell_handles = gobjects(rows, cols);
text_handles = gobjects(rows, cols);  % para mostrar índices
for i = 1:cols
    for j = 1:rows
        x = x_edges(i);
        y = y_edges(j);
        w = cell_widths(i);
        h = cell_heights(j);
        cell_handles(j, i) = rectangle('Position', [x, y, w, h], ...
            'FaceColor', [1 1 1], 'EdgeColor', [0.85 0.85 0.85]);

        % === Mostramos los índices si está activado ===
        if show_cell_indices
            bin_idx = (j - 1) * cols + (i - 1);  
            cx = x + w/2;
            cy = y + h/2;
            text_handles(j, i) = text(cx, cy, num2str(bin_idx), ...
                'HorizontalAlignment','center', 'VerticalAlignment','middle', ...
                'FontSize', 8, 'Color', [0.2 0.2 0.2]);
        end
    end
end


% === Define soft colors for class coloring ===
white = [1, 1, 1];
black = [0, 0, 0];
green_soft = [0.45 0.85 0.45];
blue_soft  = [0.45 0.45 0.85];
red_soft   = [0.85 0.45 0.45];
yellow = [1, 0.75, 0.1];  % used for estimated source
dark_gray  = [0.3 0.3 0.3];  % used for walls and obstacles

% === Draw walls ===
rectangle('Position', [-wall_thickness, -wall_thickness, 16 + wall_thickness, wall_thickness], 'FaceColor', dark_gray);
rectangle('Position', [0, 4, 16 + wall_thickness, wall_thickness], 'FaceColor', dark_gray);
rectangle('Position', [-wall_thickness, 0, wall_thickness, 4 + wall_thickness], 'FaceColor', dark_gray);
rectangle('Position', [16, 0 - wall_thickness, wall_thickness, 4 + wall_thickness], 'FaceColor', dark_gray);

% === Draw obstacles ===
rectangle('Position', [8.0, 0.0, 0.5, 2.0], 'FaceColor', dark_gray);
rectangle('Position', [4.5, 2.0, 0.5, 2.0], 'FaceColor', dark_gray);

% === Plot ground truth gas source (black star) ===
plot(source_x, source_y, 'p', 'MarkerSize', 16, ...
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');

% === Initialize robot and goal markers ===
robot_markers = gobjects(1,3);
goal_markers = gobjects(1,3);
for r = 1:3
    robot_markers(r) = plot(NaN, NaN, 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', robot_colors{r}, 'MarkerEdgeColor', 'k');
    goal_markers(r) = plot(NaN, NaN, 'x', 'MarkerSize', 10, ...
        'LineWidth', 2, 'Color', black);
end

% === Create video writer object ===
output_video_path = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/videos/animation.avi';
v = VideoWriter(output_video_path, 'Motion JPEG AVI');
v.FrameRate = video_repr_vel;
open(v);

% === Prepare iteration variables ===
max_steps = max(cellfun(@(p) size(p,1), positions));
goal_idx = ones(1, 3);
waiting = false(1,3);
last_pos = zeros(3, 2);
t_idx = ones(1, 3);
done = false(1, 3);

% === Animation loop ===
for frame_t = 1:10000
    all_reached = true;

    for r = 1:3
        if ~done(r) && t_idx(r) <= size(positions{r}, 1)
            curr_pos = positions{r}(t_idx(r), :);

            % Check if robot has reached its current goal
            if goal_idx(r) <= size(goal_positions{r}, 1)
                gx = goal_positions{r}(goal_idx(r), 1);
                gy = goal_positions{r}(goal_idx(r), 2);

                tol = 0.15;  % tolerancia para considerar que el robot llegó al goal
                dist = norm(curr_pos - [gx, gy]);
                if dist < tol
                    waiting(r) = true;
                    reached_goals(r) = 1;
                else
                    waiting(r) = false;
                    reached_goals(r) = 0;
                end

                
            end

            if ~waiting(r)
                set(robot_markers(r), 'XData', curr_pos(1), 'YData', curr_pos(2));
                last_pos(r, :) = curr_pos;
                t_idx(r) = t_idx(r) + repr_vel;
                if t_idx(r) > size(positions{r}, 1)
                    t_idx(r) = size(positions{r}, 1);
                    done(r) = true;
                end
            else
                set(robot_markers(r), 'XData', last_pos(r, 1), 'YData', last_pos(r, 2));
            end

            if goal_idx(r) <= size(goal_positions{r}, 1)
                set(goal_markers(r), 'XData', gx, 'YData', gy);
            end
        else
            done(r) = true;
        end
    end
    %disp(['Reached goals: ', mat2str(reached_goals)]);
    if all(reached_goals)
        for r = 1:3
            goal_idx(r) = goal_idx(r) + 1;
            waiting(r) = false;
        end
        reached_goals = zeros(1,3);  % reiniciar estado

        goal_update_count = goal_update_count + 1;
        %disp(['Goal indices: ', num2str(goal_idx)]);
        % === Update belief map based on update count, not position ===
        if goal_update_count <= size(belief_maps, 1)
            idx = goal_update_count;
            belief_map = belief_maps(idx, :);         % clases por celda
            intensity_map = intensity_maps(idx, :);   % intensidades por celda

        
            % === Normalizar y transformar intensidad para mejorar contraste visual ===
            min_int = min(intensity_map);
            max_int = max(intensity_map);

            if max_int > min_int
                % Normalización lineal a [0, 1]
                intensity_map = (intensity_map - min_int) / (max_int - min_int);

                % Transformación no lineal (raíz cuadrada o gamma)
                gamma = 0.4;  % puedes ajustar entre 0.3 y 0.6 para más o menos contraste
                intensity_map = intensity_map .^ gamma;
            else
                intensity_map = zeros(size(intensity_map));
            end


            fprintf('✅ Intensidad normalizada: min=%.3f max=%.3f\n', ...
                min(intensity_map), max(intensity_map));

            % Validación rápida
            fprintf('Frame %d — Intensity range: %.3f to %.3f\n', ...
                idx, min(intensity_map), max(intensity_map));

            for col = 1:cols
                for row = 1:rows
                    map_idx = (row - 1) * cols + col;
                    value = belief_map(map_idx);      % clase 0 a 3
                    intensity = intensity_map(map_idx);  % debe estar entre 0 y 1

                    % Si intensidad está fuera de rango, forzarla (por seguridad)
                    intensity = max(0, min(1, intensity));

                    % Selección de color base
                    switch value
                        case 0, base_color = white;
                        case 1, base_color = red_soft;
                        case 2, base_color = blue_soft;
                        case 3, base_color = green_soft;
                        otherwise, base_color = white;
                    end

                    % Mezcla de color con blanco según intensidad
                    blended_color = base_color * intensity + white * (1 - intensity);

                    % Aplicar color a la celda
                    set(cell_handles(row, col), 'FaceColor', blended_color);
                end
            end

            title(sprintf('FSM cycle: %d', idx), 'FontSize', 14, 'FontWeight', 'bold');
        end

    end


    % === Pause mechanism ===
    drawnow;
    while is_paused
        pause(0.1);  % wait while paused
    end

    pause(0.00001);
    frame = getframe(gcf);
    writeVideo(v, frame);

    if all(done)
        break;
    end
end


% === Plot estimated source position ===
expected_path = fullfile(exp_folder, '103', 'expected_a_posteriori.csv');
expected_data = readmatrix(expected_path);
estimated_x = expected_data(end, 2);
estimated_y = expected_data(end, 3);
plot(estimated_x, estimated_y, 'p', 'MarkerSize', 16, ...
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', yellow);

% Save final frame and close
frame = getframe(gcf);
writeVideo(v, frame);
close(v);
disp(['Video saved to: ', output_video_path]);

% === Callback function for spacebar press ===
function keypress_callback(~, event)
    if strcmp(event.Key, 'space')
        global is_paused;
        is_paused = ~is_paused;
        if is_paused
            disp('⏸️  Animation paused. Press [space] again to resume.');
        else
            disp('▶️  Animation resumed.');
        end
    end
end
