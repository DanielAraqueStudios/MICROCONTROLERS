% Código para la regresión de un sensor de luminosidad (fotorresistor)
% El fotorresistor idealmente marca 3.3V en oscuridad total y 0V en iluminación total

% Limpieza previa
clear all;
close all;
clc;

%% Gráfica del comportamiento ideal
figure(1);
% Creamos un vector de iluminación (0 = oscuridad total, 100 = iluminación total)
iluminacion_ideal = 0:1:100;
% La tensión decrece linealmente desde 3.3V (oscuridad) hasta 0V (iluminación total)
tension_ideal = 3.3 * (1 - iluminacion_ideal/100);

% Graficamos el comportamiento ideal
plot(iluminacion_ideal, tension_ideal, 'b-', 'LineWidth', 2);
grid on;
title('Comportamiento Ideal del Fotorresistor');
xlabel('Nivel de Iluminación (%)');
ylabel('Tensión (V)');
xlim([0 100]);
ylim([0 3.5]);
legend('Modelo Ideal');

%% Gráfica con datos reales (simulados)
% Estos son datos ejemplares que podrías reemplazar con tus mediciones reales
% Suponemos algunas mediciones con cierta dispersión

% Datos de iluminación (por ejemplo, medidos con un luxómetro)
iluminacion_real = [5, 15, 25, 35, 45, 55, 65, 75, 85, 95];
% Tensiones correspondientes medidas en el fotorresistor (con algo de ruido)
tension_real = [3.1, 2.8, 2.5, 2.2, 1.9, 1.5, 1.2, 0.9, 0.5, 0.2];

% Realizamos la regresión lineal
coeficientes = polyfit(iluminacion_real, tension_real, 1);
pendiente = coeficientes(1);
intercepto = coeficientes(2);

% Creamos el modelo lineal
iluminacion_modelo = 0:1:100;
tension_modelo = pendiente * iluminacion_modelo + intercepto;

% Creamos la segunda figura para los datos reales
figure(2);
% Graficamos los puntos de datos
scatter(iluminacion_real, tension_real, 100, 'r', 'filled', 'MarkerEdgeColor', 'k');
hold on;
% Graficamos la línea de regresión
plot(iluminacion_modelo, tension_modelo, 'b-', 'LineWidth', 2);
grid on;
title('Regresión Lineal del Fotorresistor con Datos Reales');
xlabel('Nivel de Iluminación (%)');
ylabel('Tensión (V)');
xlim([0 100]);
ylim([0 3.5]);
legend('Datos Medidos', 'Modelo Lineal Ajustado');

% Mostramos la ecuación de la regresión en la gráfica de manera más visible
% Formato de la ecuación con 2 decimales
if pendiente < 0
    equation = ['V = ' num2str(intercepto, '%.2f') ' - ' num2str(abs(pendiente), '%.2f') ' × L'];
else
    equation = ['V = ' num2str(intercepto, '%.2f') ' + ' num2str(pendiente, '%.2f') ' × L'];
end

% Añadimos un cuadro de texto con la ecuación
annotation('textbox', [0.15, 0.15, 0.3, 0.1], 'String', equation, ...
           'FitBoxToText', 'on', 'BackgroundColor', [0.9 0.9 0.9], ...
           'FontSize', 14, 'FontWeight', 'bold', 'EdgeColor', 'blue', ...
           'LineWidth', 2, 'HorizontalAlignment', 'center');

% Calculamos el coeficiente de determinación (R²)
yMean = mean(tension_real);
SST = sum((tension_real - yMean).^2);
SSR = sum((tension_real - (pendiente * iluminacion_real + intercepto)).^2);
R2 = 1 - (SSR/SST);

% Mostramos el R² en la gráfica
R2_text = ['R² = ' num2str(R2, '%.4f')];
text(60, 3, R2_text, 'FontSize', 12, 'Color', 'blue', 'FontWeight', 'bold');

fprintf('Ecuación del modelo: V = %.2f + (%.2f × L)\n', intercepto, pendiente);
fprintf('donde V es la tensión y L es el nivel de iluminación\n');
fprintf('Coeficiente de determinación (R²): %.4f\n', R2);

%% Comparación entre modelo ideal y real
figure(3);
plot(iluminacion_ideal, tension_ideal, 'b-', 'LineWidth', 2);
hold on;
plot(iluminacion_modelo, tension_modelo, 'r-', 'LineWidth', 2);
scatter(iluminacion_real, tension_real, 100, 'r', 'filled', 'MarkerEdgeColor', 'k');
grid on;
title('Comparación entre Modelo Ideal y Real del Fotorresistor');
xlabel('Nivel de Iluminación (%)');
ylabel('Tensión (V)');
xlim([0 100]);
ylim([0 3.5]);
legend('Modelo Ideal', 'Modelo Lineal Ajustado', 'Datos Medidos');

% Añadimos también la ecuación a esta gráfica
annotation('textbox', [0.15, 0.15, 0.3, 0.1], 'String', equation, ...
           'FitBoxToText', 'on', 'BackgroundColor', [0.9 0.9 0.9], ...
           'FontSize', 14, 'FontWeight', 'bold', 'EdgeColor', 'red', ...
           'LineWidth', 2, 'HorizontalAlignment', 'center');