% Caracterización de sensores: Sharp y Fotorresistor
% Este código muestra la caracterización de ambos sensores en un solo archivo
% con sus respectivas ecuaciones

% Limpieza previa
clear all;
close all;
clc;

% Creamos una figura con dos subplots
figure('Name', 'Caracterización de Sensores', 'Position', [100, 100, 1200, 500]);

%% CARACTERIZACIÓN DEL SENSOR SHARP (GP2Y0A21YK)
subplot(1, 2, 1);

% Datos del sensor Sharp (Distancia en cm vs Voltaje)
% Estos son datos típicos del sensor GP2Y0A21YK, ajustar según mediciones reales
distancia_cm = [10, 15, 20, 25, 30, 40, 50, 60, 70, 80];
voltaje_sharp = [2.6, 2.1, 1.5, 1.2, 1.0, 0.7, 0.6, 0.5, 0.4, 0.4];

% Ajuste de modelo hiperbólico: V = a/(d+b) + c
% Este modelo es típico para los sensores Sharp de distancia
modelfun = @(b,x) b(1)./(x+b(2)) + b(3);
beta0 = [30, 1, 0]; % Valores iniciales para a, b, c
[beta,~,~,~,~] = nlinfit(distancia_cm, voltaje_sharp, modelfun, beta0);

% Creamos curva ajustada con el modelo
distancia_modelo = linspace(8, 85, 100);
voltaje_modelo_sharp = modelfun(beta, distancia_modelo);

% Graficamos los datos y el modelo
scatter(distancia_cm, voltaje_sharp, 100, 'r', 'filled', 'MarkerEdgeColor', 'k');
hold on;
plot(distancia_modelo, voltaje_modelo_sharp, 'b-', 'LineWidth', 2);
grid on;
title('Caracterización del Sensor Sharp', 'FontSize', 14);
xlabel('Distancia (cm)', 'FontSize', 12);
ylabel('Voltaje (V)', 'FontSize', 12);
xlim([0 90]);
ylim([0 3]);
legend('Datos Medidos', 'Modelo Ajustado', 'Location', 'NorthEast');

% Creamos la ecuación para mostrar en la gráfica
eq_sharp = sprintf('V = %.2f/(d + %.2f) + %.2f', beta(1), beta(2), beta(3));
% Versión despejada para distancia
eq_sharp_dist = sprintf('d = %.2f/(V - %.2f) - %.2f', beta(1), beta(3), beta(2));

% Mostramos la ecuación en la gráfica
annotation('textbox', [0.02, 0.85, 0.25, 0.08], 'String', eq_sharp, ...
           'FitBoxToText', 'on', 'BackgroundColor', [0.9 0.9 0.9], ...
           'FontSize', 12, 'FontWeight', 'bold', 'EdgeColor', 'blue', ...
           'LineWidth', 1.5, 'HorizontalAlignment', 'center');

annotation('textbox', [0.02, 0.75, 0.25, 0.08], 'String', eq_sharp_dist, ...
           'FitBoxToText', 'on', 'BackgroundColor', [0.9 0.9 0.9], ...
           'FontSize', 12, 'FontWeight', 'bold', 'EdgeColor', 'blue', ...
           'LineWidth', 1.5, 'HorizontalAlignment', 'center');

%% CARACTERIZACIÓN DEL FOTORRESISTOR
subplot(1, 2, 2);

% Datos de iluminación vs voltaje para el fotorresistor
iluminacion_real = [5, 15, 25, 35, 45, 55, 65, 75, 85, 95];
tension_real = [3.1, 2.8, 2.5, 2.2, 1.9, 1.5, 1.2, 0.9, 0.5, 0.2];

% Realizamos la regresión lineal
coeficientes = polyfit(iluminacion_real, tension_real, 1);
pendiente = coeficientes(1);
intercepto = coeficientes(2);

% Creamos el modelo lineal
iluminacion_modelo = 0:1:100;
tension_modelo = pendiente * iluminacion_modelo + intercepto;

% Graficamos los datos y el modelo
scatter(iluminacion_real, tension_real, 100, 'r', 'filled', 'MarkerEdgeColor', 'k');
hold on;
plot(iluminacion_modelo, tension_modelo, 'b-', 'LineWidth', 2);
grid on;
title('Caracterización del Fotorresistor', 'FontSize', 14);
xlabel('Nivel de Iluminación (%)', 'FontSize', 12);
ylabel('Voltaje (V)', 'FontSize', 12);
xlim([0 100]);
ylim([0 3.5]);
legend('Datos Medidos', 'Modelo Lineal Ajustado', 'Location', 'NorthEast');

% Calculamos el coeficiente de determinación (R²)
yMean = mean(tension_real);
SST = sum((tension_real - yMean).^2);
SSR = sum((tension_real - (pendiente * iluminacion_real + intercepto)).^2);
R2 = 1 - (SSR/SST);

% Formato de la ecuación del fotorresistor con 2 decimales
if pendiente < 0
    equation_foto = ['V = ' num2str(intercepto, '%.2f') ' - ' num2str(abs(pendiente), '%.2f') ' × L'];
    % Versión despejada para iluminación
    equation_foto_ilum = ['L = ' num2str((intercepto/abs(pendiente)), '%.2f') ' - ' num2str((1/abs(pendiente)), '%.2f') ' × V'];
else
    equation_foto = ['V = ' num2str(intercepto, '%.2f') ' + ' num2str(pendiente, '%.2f') ' × L'];
    % Versión despejada para iluminación
    equation_foto_ilum = ['L = ' num2str((-intercepto/pendiente), '%.2f') ' + ' num2str((1/pendiente), '%.2f') ' × V'];
end

% Mostramos la ecuación en la gráfica
annotation('textbox', [0.52, 0.85, 0.25, 0.08], 'String', equation_foto, ...
           'FitBoxToText', 'on', 'BackgroundColor', [0.9 0.9 0.9], ...
           'FontSize', 12, 'FontWeight', 'bold', 'EdgeColor', 'blue', ...
           'LineWidth', 1.5, 'HorizontalAlignment', 'center');

annotation('textbox', [0.52, 0.75, 0.25, 0.08], 'String', equation_foto_ilum, ...
           'FitBoxToText', 'on', 'BackgroundColor', [0.9 0.9 0.9], ...
           'FontSize', 12, 'FontWeight', 'bold', 'EdgeColor', 'blue', ...
           'LineWidth', 1.5, 'HorizontalAlignment', 'center');

% Mostramos el R² en la gráfica
R2_text = ['R² = ' num2str(R2, '%.4f')];
annotation('textbox', [0.52, 0.65, 0.12, 0.08], 'String', R2_text, ...
           'FitBoxToText', 'on', 'BackgroundColor', [0.9 0.9 0.9], ...
           'FontSize', 12, 'FontWeight', 'bold', 'EdgeColor', 'blue', ...
           'LineWidth', 1.5, 'HorizontalAlignment', 'center');

%% Mostrar estadísticas en la consola
fprintf('---- CARACTERIZACIÓN DEL SENSOR SHARP ----\n');
fprintf('Ecuación del modelo: V = %.4f/(d + %.4f) + %.4f\n', beta(1), beta(2), beta(3));
fprintf('Ecuación despejada: d = %.4f/(V - %.4f) - %.4f\n\n', beta(1), beta(3), beta(2));

fprintf('---- CARACTERIZACIÓN DEL FOTORRESISTOR ----\n');
fprintf('Ecuación del modelo: V = %.4f + (%.4f × L)\n', intercepto, pendiente);
fprintf('Ecuación despejada: L = %.4f + (%.4f × V)\n', (-intercepto/pendiente), (1/pendiente));
fprintf('Coeficiente de determinación (R²): %.4f\n', R2);

% Ajustar la posición de los subplots para mejor visualización
set(gcf, 'Position', [100, 100, 1200, 500]);