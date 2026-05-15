% Crear figura
figure;
hold on;
axis equal;

% Grosor de pared
wall_thickness = 0.1;

% Ajustar el axis para incluir paredes exteriores
axis([-wall_thickness 16 + wall_thickness -wall_thickness 4 + wall_thickness]);

title('Map 9');
xlabel('X (m)');
ylabel('Y (m)');

% Dibujar cuadrícula de fondo (20x20)
cols = 20;
rows = 20;
cell_width = 16 / cols;
cell_height = 4 / rows;
cell_handles = gobjects(rows, cols);

green_soft = [0.9 1.0 0.9];   % Verde claro
blue_soft  = [0.9 0.9 1.0];   % Azul claro
red_soft   = [1.0 0.9 0.9];   % Rojo claro
dark_gray = [0.3 0.3 0.3];    % Gris oscuro


for i = 1:cols
    for j = 1:rows
        x = (i - 1) * cell_width;
        y = (j - 1) * cell_height;
        cell_handles(j, i) = rectangle('Position', [x, y, cell_width, cell_height], ...
            'FaceColor', red_soft, 'EdgeColor', [0.85 0.85 0.85]);
    end
end

% Dibujar paredes por fuera
rectangle('Position', [-wall_thickness, -wall_thickness, 16 + wall_thickness, wall_thickness], ...
          'FaceColor', dark_gray, 'EdgeColor', 'k');  % inferior
rectangle('Position', [0, 4, 16 + wall_thickness, wall_thickness], ...
          'FaceColor', dark_gray, 'EdgeColor', 'k');  % superior
rectangle('Position', [-wall_thickness, 0, wall_thickness, 4 + wall_thickness], ...
          'FaceColor', dark_gray, 'EdgeColor', 'k');  % izquierda
rectangle('Position', [16, 0 - wall_thickness, wall_thickness, 4 + wall_thickness], ...
          'FaceColor', dark_gray, 'EdgeColor', 'k');  % derecha

% Obstáculo 1 invertido
x1_inv = 16 - (8.0 + 0.5);
y1_inv = 4 - (0.0 + 2.0);
rectangle('Position', [x1_inv, y1_inv, 0.5, 2.0], ...
          'FaceColor', dark_gray, 'EdgeColor', dark_gray);

% Obstáculo 2 invertido
x2_inv = 16 - (4.5 + 0.5);
y2_inv = 4 - (2.0 + 2.0);
rectangle('Position', [x2_inv, y2_inv, 0.5, 2.0], ...
          'FaceColor', dark_gray, 'EdgeColor', dark_gray);
      

