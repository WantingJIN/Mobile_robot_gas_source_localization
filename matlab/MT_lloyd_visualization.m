% === Configuración inicial ===
clc; clear;

% === Buscar automáticamente la carpeta del experimento y archivo belief ===
base_path = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/cluster_analysis';
d = dir(base_path);
subfolder = d([d.isdir] & ~startsWith({d.name}, '.'));  % omitir carpetas ocultas como '.' y '..'

if isempty(subfolder)
    error('❌ No se encontró ninguna subcarpeta dentro de cluster_analysis');
end

exp_folder = fullfile(base_path, subfolder(1).name);
belief_path = fullfile(exp_folder, '101', 'posterior_pdf_3D_cluster_lloyd.csv');

if ~isfile(belief_path)
    error('❌ No se encontró el archivo belief en: %s', belief_path);
end

% === Cargar belief map (clasificación) ===
belief_data = readmatrix(belief_path);
belief_maps = belief_data(:, 3:end);  % ignoramos las columnas X,Y
num_frames = size(belief_maps, 1);

% === Parámetros del mapa ===
cols = 20; rows = 20;
x_edges = linspace(3, 12.5, cols + 1);
y_edges = linspace(0.16, 3.8, rows + 1);
cell_widths = diff(x_edges);
cell_heights = diff(y_edges);

% === Colores por clase ===
white = [1 1 1];
red_soft   = [0.85 0.45 0.45];
blue_soft  = [0.45 0.45 0.85];
green_soft = [0.45 0.85 0.45];

% === Crear figura y mapa base ===
figure('Position', [100, 100, 1200, 400]);
axis equal;
hold on;
axis([-0.1 16.1 -0.1 4.1]);
xlabel('X (m)');
ylabel('Y (m)');
title('FSM cycle: 0');

% === Dibujar celdas ===
cell_patches = gobjects(rows, cols);
for i = 1:cols
    for j = 1:rows
        x = x_edges(i);
        y = y_edges(j);
        w = cell_widths(i);
        h = cell_heights(j);
        xv = [x, x + w, x + w, x];
        yv = [y, y, y + h, y + h];
        cell_patches(j, i) = patch(xv, yv, white, 'EdgeColor', 'none');
    end
end

% === Dibujar paredes ===
wall_thickness = 0.1;
dark_gray = [0.3 0.3 0.3];
rectangle('Position', [-wall_thickness, -wall_thickness, 16 + wall_thickness, wall_thickness], 'FaceColor', dark_gray);
rectangle('Position', [0, 4, 16 + wall_thickness, wall_thickness], 'FaceColor', dark_gray);
rectangle('Position', [-wall_thickness, 0, wall_thickness, 4 + wall_thickness], 'FaceColor', dark_gray);
rectangle('Position', [16, 0 - wall_thickness, wall_thickness, 4 + wall_thickness], 'FaceColor', dark_gray);

% === Dibujar obstáculos ===
rectangle('Position', [8.0, 0.0, 0.5, 2.0], 'FaceColor', dark_gray);
rectangle('Position', [4.5, 2.0, 0.5, 2.0], 'FaceColor', dark_gray);

% Add key press callback for pausing
set(gcf, 'KeyPressFcn', @keypress_callback);
global is_paused;
is_paused = false; 

% === Animación del belief map ===
for idx = 1:num_frames
    belief_map = belief_maps(idx, :);

    for col = 1:cols
        for row = 1:rows
            map_idx = (row - 1) * cols + col;
            value = belief_map(map_idx);

            switch value
                case 0, base_color = white;
                case 1, base_color = red_soft;
                case 2, base_color = blue_soft;
                case 3, base_color = green_soft;
                otherwise, base_color = white;
            end

            set(cell_patches(row, col), 'FaceColor', base_color);
        end
    end

    fsm_cycle = floor((idx - 1) / 10) + 1;
    iter_in_cycle = mod(idx - 1, 10) + 1;
    title(sprintf('FSM cycle: %d (%d)', fsm_cycle, iter_in_cycle), 'FontSize', 14, 'FontWeight', 'bold');
    
    while is_paused
        pause(0.1);
    end
    drawnow;
    
    if mod(idx, 10) == 0
        pause(3);
    else
        pause(0.3);
    end
   
end



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