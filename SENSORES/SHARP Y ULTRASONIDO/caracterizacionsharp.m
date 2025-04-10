% Datos experimentales del sensor Sharp 2Y0A21
distancia_cm = [10 10.5 11 11.5 12 12.5 13 13.5 14 14.5 15 15.5 16 16.5 17 17.5 18 18.5 19 19.5 20 20.5 21 21.5 22 22.5 23 23.5 24 24.5 25];
voltaje = [2.17 2.08 1.98 1.89 1.8 1.75 1.7 1.64 1.55 1.48 1.5 1.47 1.41 1.39 1.36 1.36 1.31 1.33 1.34 1.26 1.2 1.16 1.19 1.15 1.13 1.09 1.11 1.09 1.07 1.05 1.03];

% Aplicar transformación logarítmica para realizar regresión lineal (log(y) = log(a) + b*log(x))
log_distancia = log(distancia_cm);
log_voltaje = log(voltaje);

% Ajuste de regresión lineal en el espacio logarítmico
coeficientes = polyfit(log_voltaje, log_distancia, 1);

% Extraer los parámetros de la regresión potencial
b = coeficientes(1);  % Exponente
a = exp(coeficientes(2));  % Coeficiente (antilogaritmo)

% Calcular coeficiente de determinación (R²)
y_estimado_log = polyval(coeficientes, log_voltaje);
SST = sum((log_distancia - mean(log_distancia)).^2);
SSE = sum((log_distancia - y_estimado_log).^2);
R_cuadrado = 1 - SSE/SST;

% Generar puntos para la curva de ajuste
voltaje_ajuste = linspace(min(voltaje), max(voltaje), 100);
distancia_ajuste = a * voltaje_ajuste.^b;

% Visualización
figure('Name', 'Caracterización del Sensor Sharp 2Y0A21', 'NumberTitle', 'off');

% Gráfico de dispersión de los datos originales
scatter(voltaje, distancia_cm, 50, 'filled', 'MarkerFaceColor', [0 0.4470 0.7410]);
hold on;

% Curva de ajuste
plot(voltaje_ajuste, distancia_ajuste, 'r-', 'LineWidth', 2);

% Etiquetas y título
xlabel('Voltaje (V)', 'FontSize', 12);
ylabel('Distancia (cm)', 'FontSize', 12);
title('Caracterización del Sensor Sharp 2Y0A21', 'FontSize', 14);
grid on;

% Mostrar ecuación de regresión y R² en la gráfica
equation = sprintf('d = %.2f × V^{%.3f}', a, b);
rsquared = sprintf('R² = %.4f', R_cuadrado);
text(1.8, 23, equation, 'FontSize', 12);
text(1.8, 21, rsquared, 'FontSize', 12);

% Añadir leyenda
legend('Datos experimentales', 'Modelo de regresión potencial', 'Location', 'best');

% Ajustar la visualización
axis tight;
box on;

% Mostrar la ecuación y R² en la consola
fprintf('Ecuación del modelo potencial: d = %.2f × V^(%.3f)\n', a, b);
fprintf('Coeficiente de determinación (R²): %.4f\n', R_cuadrado);

% Función predicción
% Crear una función anónima que permita estimar la distancia a partir del voltaje
distancia_prediccion = @(v) a * v.^b;

% Ejemplo: calcular distancia para un voltaje de 1.5V
voltaje_ejemplo = 1.5;
distancia_calculada = distancia_prediccion(voltaje_ejemplo);
fprintf('\nPara un voltaje de %.2f V, la distancia estimada es %.2f cm\n', voltaje_ejemplo, distancia_calculada);