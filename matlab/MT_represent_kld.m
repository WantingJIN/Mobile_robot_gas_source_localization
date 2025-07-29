% Leer el archivo
data = readmatrix('C:\Users\Marc\Downloads\kld_value8.csv');

gray = [0.9, 0.9, 0.9];

% Extraer columnas
kld = data(:, 2);                    % Segunda columna: KLD
tiempo = data(:, end);              % Última columna: Tiempo

% Crear figura
figure('Color', 'w');               % Fondo blanco (ideal para papers)
plot(tiempo, kld, 'r-', 'LineWidth', 1.5);  % Línea azul con grosor
hold on;  % Asegura que la línea no borre la curva original
% Línea horizontal en Y=1 de color gris
yline(1, '--', 'Threshold = 1', ...
    'Color', [0.3 0.3 0.3], ...
    'LabelHorizontalAlignment', 'left', ...
    'LabelVerticalAlignment', 'bottom', ...
    'FontSize', 12, 'LineWidth', 1.2);

% Etiquetas con fuente clara
xlabel('Time (s)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('KLD', 'FontSize', 14, 'FontWeight', 'bold');

% Mejoras visuales
grid on;
set(gca, ...
    'FontSize', 12, ...
    'LineWidth', 1, ...
    'Box', 'off', ...
    'TickDir', 'out', ...
    'TickLength', [0.01 0.01], ...
    'XColor', 'k', 'YColor', 'k');

% Exportar con alta resolución (opcional)
% exportgraphics(gcf, 'KLD_vs_Tiempo.png', 'Resolution', 300);  % PNG
% exportgraphics(gcf, 'KLD_vs_Tiempo.pdf');  % PDF para paper

