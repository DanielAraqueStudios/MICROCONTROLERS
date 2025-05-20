classdef STM32VoltageMonitor < handle
    properties
        % Configuración del puerto serial
        serialPort = 'COM21';  
        baudRate = 9600;     % Cambiado de 9600 a 115200 baudios
        
        % Configuración de la gráfica
        maxPoints = 500;  % Aumentado para mejor visualización
        startScroll = 50;
        scrollMargin = 5;
        
        % Objetos de MATLAB
        serialObj;
        fig;
        ax;
        linePlot;
        
        % Datos
        timePoints = [];
        voltPoints = [];
        sampleCount = 0;
        lastVolt = 0;
        
        % Constantes
        DEFAULT_ARR = 15999;
        CLOCK_FREQ = 16000000; % 16 MHz
        
        % Timer para actualización
        updateTimer;
        
        % Umbrales de respiración
        INHALE_THRESHOLD = 1.2;  % Voltaje para inhalación (0V - 1.2V)
        EXHALE_THRESHOLD = 1.2;  % Voltaje para exhalación (1.2V - 3.3V)
        lastBreathState = 1;     % 2: inhalando, 1: neutral, 0: exhalando
        
        % Conteo de respiraciones
        breathCount = 0;
        lastBreathTime = 0;
        
        % UI Elements
        warningText;  % Texto de advertencia en GUI
        warningImage; % Imagen de advertencia
        imageAxes;    % Axes para la imagen
        
        % Modificar los límites de voltaje
        VOLTAGE_MIN = 0;
        VOLTAGE_MAX = 3.3;
        
        % BPM properties - Unificación de propiedades BPM
        bpmDisplay;         % Display de BPM (reemplaza bpmText)
        peakThreshold = 0.5;    % Umbral más bajo para mejor detección
        minPeakHeight = 1.2;    % Altura mínima ajustada
        lastPeakTime = 0;
        peakTimes = [];         % Array de tiempos de picos
        bpmValue = 0;
        sampleRate = 100;       % Tasa de muestreo estimada (Hz)
        
        % BPM detection properties
        windowSize = 20;         % Tamaño de la ventana para detección
        lastValues = [];         % Buffer circular para valores
        isPeakDetected = false;  % Estado de detección de pico
        minPeakDistance = 0.2;   % Tiempo mínimo entre picos (segundos)
        
        % Additional axes
        axRespiration;    % Axes for respiration pattern
        axSweat;          % Axes for sweat level
        respirationPlot;  % Plot line for respiration
        sweatPie;         % Pie chart for sweat level
        sweatText;        % Text display for sweat percentage
        
        % Additional data
        lastVolt2 = 0;    % Respiration voltage
        lastVolt3 = 0;    % Sweat voltage
        timePoints2 = []; % For respiration
        voltPoints2 = []; % For respiration
        
        % Peak detection properties
        peakIndicator;     % Indicador visual de picos
        lastPeakValue = 0; % Valor del último pico
        
        % Filtrado de señal de respiración
        respFilterWindow = 30;    % Aumentado de 15 a 30 para más suavizado
        respBuffer = [];          % Buffer para filtrado
        smoothingFactor = 0.95;   % Aumentado de 0.8 a 0.95 para más suavizado
        lastSmoothedValue = 0;    % Último valor suavizado
        
        % Propiedades para zoom dinámico
        minVolt1 = 3.3;      % Mínimo valor ECG
        maxVolt1 = 0;        % Máximo valor ECG
        minVolt2 = 3.3;      % Mínimo valor respiración
        maxVolt2 = 0;        % Máximo valor respiración
        dynamicMargin = 0.1; % Margen para el zoom (10%)
    end
    
    methods
        function obj = STM32VoltageMonitor()
            % Inicializar la interfaz
            obj.createUI();
            
            % Configurar el puerto serial
            try
                obj.serialObj = serialport(obj.serialPort, obj.baudRate);
                configureTerminator(obj.serialObj, "LF");
                disp(['Conectado al puerto serial: ' obj.serialPort]);
            catch e
                errordlg(['Error al abrir el puerto serial: ' e.message], 'Error de conexión');
                return;
            end
            
            % Configurar timer para actualización
            obj.updateTimer = timer(...
                'ExecutionMode', 'fixedRate', ...
                'Period', 0.001, ...  % Cambiado de 0.1 a 0.001 segundos (1ms)
                'TimerFcn', @(src,event)obj.updatePlot());
            
            start(obj.updateTimer);
        end
        
        function createUI(obj)
            % Crear figura con tema oscuro
            obj.fig = figure('Name', 'Monitor Multiparámetro', ...
                'NumberTitle', 'off', ...
                'Position', [100 100 1300 900], ...  % Increased size more
                'Color', [0.1 0.1 0.1], ...         % Darker background
                'CloseRequestFcn', @(src,event)obj.closeFigure());
            
            % ECG Plot (top)
            obj.ax = axes('Parent', obj.fig, ...
                'Position', [0.15 0.57 0.75 0.35], ...
                'Color', [0.12 0.12 0.12], ...
                'XColor', [0.7 0.7 0.7], ...
                'YColor', [0.7 0.7 0.7], ...
                'GridColor', [0.2 0.2 0.2], ...
                'GridAlpha', 0.3, ...
                'YTick', 0:0.5:3.3);  % Marcas cada 0.5V
            
            title(obj.ax, 'ECG', 'Color', [0.8 0.8 0.8], 'FontWeight', 'bold');
            xlabel(obj.ax, 'Tiempo (s)', 'Color', [0.7 0.7 0.7]);
            ylabel(obj.ax, 'Voltaje (V)', 'Color', [0.7 0.7 0.7]);
            grid(obj.ax, 'on');
            ylim(obj.ax, [0 3.3]);

            % Respiration Plot (middle)
            obj.axRespiration = axes('Parent', obj.fig, ...
                'Position', [0.15 0.12 0.75 0.35], ...
                'Color', [0.12 0.12 0.12], ...
                'XColor', [0.7 0.7 0.7], ...
                'YColor', [0.7 0.7 0.7], ...
                'GridColor', [0.2 0.2 0.2], ...
                'GridAlpha', 0.3, ...
                'YTick', 1.3:0.02:1.6);  % Marcas más precisas y cercanas

            title(obj.axRespiration, 'Patrón de Respiración', 'Color', [0.8 0.8 0.8], 'FontWeight', 'bold');
            xlabel(obj.axRespiration, 'Tiempo (s)', 'Color', [0.7 0.7 0.7]);
            ylabel(obj.axRespiration, 'Voltaje (V)', 'Color', [0.7 0.7 0.7]);
            grid(obj.axRespiration, 'on');
            ylim(obj.axRespiration, [1.3 1.6]);  % Rango más estrecho para mayor zoom

            % Sweat Level Display (right) - Cambiando de barra a círculo
            obj.axSweat = polaraxes('Parent', obj.fig, ...
                'Position', [0.88 0.2 0.1 0.3], ...  % Aumentado tamaño
                'Color', [0.12 0.12 0.12]);
            
            % Configurar el gráfico polar
            hold(obj.axSweat, 'on');
            obj.axSweat.ThetaGrid = 'off';
            obj.axSweat.RGrid = 'off';
            obj.axSweat.ThetaTickLabels = {};
            obj.axSweat.RTickLabels = {};
            
            % Inicializar gráfico circular
            obj.sweatPie = polarplot(obj.axSweat, [0 0], [0 1], '-r', 'LineWidth', 3);  % Línea más gruesa
            
            title(obj.axSweat, 'CONDUCTANCIA', ...
                'Color', [1 1 1], ...
                'FontWeight', 'bold', ...
                'FontSize', 11);
            
            % Texto para mostrar microsiemens
            obj.sweatText = text(obj.axSweat, 0, 0, '0 µS', ...  % Cambiado a microsiemens
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', ...
                'Color', [1 1 1], ...
                'FontSize', 14, ...
                'FontWeight', 'bold');

            % Initialize plots with modern style
            obj.linePlot = line(obj.ax, NaN, NaN, ...
                'Color', [0.3 0.8 1], ...               % Brighter blue
                'LineWidth', 1.5);
            
            obj.respirationPlot = line(obj.axRespiration, NaN, NaN, ...
                'Color', [0.3 1 0.5], ...               % Brighter green
                'LineWidth', 1.5);
            
            % Panel para BPM con nuevo estilo
            bpmPanel = uipanel('Parent', obj.fig, ...
                'Title', 'Ritmo Cardíaco', ...
                'Position', [0.02 0.8 0.11 0.15], ...    % Adjusted position
                'BackgroundColor', [0.15 0.15 0.15], ...
                'ForegroundColor', [0.8 0.8 0.8], ...
                'HighlightColor', [0.3 0.3 0.3], ...
                'BorderType', 'line', ...
                'FontWeight', 'bold');
            
            % Display de BPM con estilo moderno
            obj.bpmDisplay = uicontrol('Parent', bpmPanel, ...  % Cambiado de bpmText a bpmDisplay
                'Style', 'text', ...
                'String', '0 BPM', ...
                'Position', [10 10 120 40], ...
                'BackgroundColor', [0.12 0.12 0.12], ...
                'ForegroundColor', [0 1 0], ...
                'FontSize', 20, ...
                'FontWeight', 'bold');
            
            % Después de crear el panel BPM, añadir el indicador de picos
            obj.peakIndicator = uipanel('Parent', bpmPanel, ...
                'Position', [100 35 15 15], ...  % Ajusta la posición según necesites
                'BackgroundColor', [0.2 0.2 0.2], ...
                'BorderType', 'none');
            
            % Actualizar estilo del texto de advertencia
            obj.warningText = uicontrol('Style', 'text', ...
                'Position', [50 550 200 30], ...
                'String', '¡Cálmate parce, respira más despacio!', ...
                'BackgroundColor', [0.8 0.2 0.2], ...
                'ForegroundColor', [1 1 1], ...
                'FontSize', 12, ...
                'Visible', 'off');
            
            % Crear axes para la imagen
            obj.imageAxes = axes('Parent', obj.fig, ...
                'Position', [0.05 0.6 0.2 0.2], ... % Ajusta estas coordenadas según necesites
                'Visible', 'off');
            
            % Cargar y configurar la imagen
            try
                img = imread('images/calmate.png'); % Asegúrate de tener esta imagen en la carpeta
                obj.warningImage = image(obj.imageAxes, img);
                axis(obj.imageAxes, 'off');
                obj.imageAxes.Visible = 'off';
            catch
                warning('No se pudo cargar la imagen de advertencia');
            end
        end
        
        function sendCommand(obj, cmd)
            try
                writeline(obj.serialObj, cmd);
            catch e
                errordlg(['Error enviando comando: ' e.message], 'Error');
            end
        end

        function updatePlot(obj)
            if obj.serialObj.NumBytesAvailable > 0
                line = readline(obj.serialObj);
                obj.parseData(line);
                
                if obj.lastVolt ~= 0
                    obj.sampleCount = obj.sampleCount + 1;
                    currentTime = obj.sampleCount / obj.sampleRate; % Tiempo en segundos
                    
                    % Actualizar buffer circular para detección de picos
                    obj.lastValues(end+1) = obj.lastVolt;
                    if length(obj.lastValues) > obj.windowSize
                        obj.lastValues = obj.lastValues(end-obj.windowSize+1:end);
                    end
                    
                    % Detección de picos mejorada (reemplazar la sección existente)
                    if length(obj.lastValues) >= 3
                        middleValue = obj.lastValues(ceil(end/2));
                        meanValue = mean(obj.lastValues);
                        
                        % Detector de cruces por umbral relativo
                        isAboveThreshold = middleValue > (meanValue * 1.2); % 20% por encima de la media
                        
                        % Detector de picos usando tiempo
                        if isAboveThreshold && ~obj.isPeakDetected && ...
                           (currentTime - obj.lastPeakTime) > 0.4  % Mínimo 400ms entre picos
                            
                            % Marcar pico
                            obj.peakTimes(end+1) = currentTime;
                            obj.lastPeakTime = currentTime;
                            obj.isPeakDetected = true;
                            obj.lastPeakValue = middleValue;
                            
                            % Efecto visual del indicador
                            set(obj.peakIndicator, 'BackgroundColor', [1 0 0]);  % Rojo cuando detecta
                            pause(0.05);  % Breve destello
                            set(obj.peakIndicator, 'BackgroundColor', [0.2 0.2 0.2]);  % Volver a oscuro
                            
                            % Calcular BPM usando solo tiempos
                            if length(obj.peakTimes) > 1
                                % Usar los últimos 4 intervalos como máximo
                                lastPeaks = obj.peakTimes(max(1, end-4):end);
                                intervals = diff(lastPeaks);
                                avgInterval = mean(intervals);
                                
                                % Convertir a BPM y ajustar el valor
                                newBPM = round(60 / avgInterval) - 20;  % Restar 20 al valor calculado
                                
                                % Filtrar valores no fisiológicos
                                if newBPM >= 40 && newBPM <= 180
                                    obj.bpmValue = newBPM;
                                    set(obj.bpmDisplay, 'String', sprintf('%d BPM', obj.bpmValue));
                                    
                                    % Actualizar color según rango ajustado
                                    if obj.bpmValue < 60
                                        set(obj.bpmDisplay, 'ForegroundColor', [1 1 0]);  % Amarillo - Bradicardia
                                    elseif obj.bpmValue > 100
                                        set(obj.bpmDisplay, 'ForegroundColor', [1 0 0]);  % Rojo - Taquicardia
                                    else
                                        set(obj.bpmDisplay, 'ForegroundColor', [0 1 0]);  % Verde - Normal
                                    end
                                end
                            end
                            
                            % Mantener solo los últimos 10 tiempos de pico
                            if length(obj.peakTimes) > 10
                                obj.peakTimes = obj.peakTimes(end-9:end);
                            end
                        elseif ~isAboveThreshold
                            obj.isPeakDetected = false;
                        end
                    end
                    
                    % Suavizado mejorado de la señal de respiración
                    obj.respBuffer(end+1) = 3.3 - obj.lastVolt2;  % Invertir y escalar a rango positivo
                    if length(obj.respBuffer) > obj.respFilterWindow
                        obj.respBuffer = obj.respBuffer(end-obj.respFilterWindow+1:end);
                    end
                    
                    if ~isempty(obj.respBuffer)
                        % Aplicar filtro exponencial mejorado con media móvil ponderada
                        currentMean = mean(obj.respBuffer);
                        if isempty(obj.lastSmoothedValue)
                            obj.lastSmoothedValue = currentMean;
                        end
                        
                        smoothedValue = obj.smoothingFactor * obj.lastSmoothedValue + ...
                                      (1 - obj.smoothingFactor) * currentMean;
                        obj.lastSmoothedValue = smoothedValue;
                        
                        % Actualizar la gráfica con el valor suavizado
                        obj.timePoints2(end+1) = obj.sampleCount;
                        obj.voltPoints2(end+1) = smoothedValue;
                        
                        if length(obj.timePoints2) > obj.maxPoints
                            obj.timePoints2 = obj.timePoints2(end-obj.maxPoints+1:end);
                            obj.voltPoints2 = obj.voltPoints2(end-obj.maxPoints+1:end);
                        end
                        
                        set(obj.respirationPlot, 'XData', obj.timePoints2, 'YData', obj.voltPoints2);
                    end
                    
                    % Actualizar gráficas
                    obj.timePoints(end+1) = obj.sampleCount;
                    obj.voltPoints(end+1) = obj.lastVolt;
                    
                    % Buffer management
                    if length(obj.timePoints) > obj.maxPoints + obj.scrollMargin
                        obj.timePoints = obj.timePoints(end-obj.maxPoints-obj.scrollMargin+1:end);
                        obj.voltPoints = obj.voltPoints(end-obj.maxPoints-obj.scrollMargin+1:end);
                    end
                    
                    % Update plots
                    set(obj.linePlot, 'XData', obj.timePoints, 'YData', obj.voltPoints);
                    
                    % Actualizar visualización de sudoración
                    sweatPercentage = obj.lastVolt3 * 15.15;  % Convertir a porcentaje
                    theta = linspace(0, 2*pi*sweatPercentage/50, 50);  % Mapear a 2π
                    set(obj.sweatPie, 'ThetaData', theta, 'RData', ones(size(theta)));
                    set(obj.sweatText, 'String', sprintf('%.1f µS', sweatPercentage));
                    
                    % Mostrar valores en terminal
                    fprintf('ADC1=%.2fV, ADC2=%.2fV, ADC3=%.2fV | BPM=%d\n', ...
                        obj.lastVolt, obj.lastVolt2, obj.lastVolt3, obj.bpmValue);
                    
                    if obj.sampleCount > obj.startScroll
                        xlim(obj.ax, [max(1, obj.sampleCount - obj.maxPoints), obj.sampleCount]);
                        xlim(obj.axRespiration, [max(1, obj.sampleCount - obj.maxPoints), obj.sampleCount]);
                    end
                    
                    drawnow limitrate;  % Limitar tasa de actualización
                end
            end
        end
        
        function parseData(obj, line)
            try
                line = strtrim(line);
                parts = strsplit(line, ',');
                
                for i = 1:length(parts)
                    if startsWith(parts{i}, 'V1=')
                        obj.lastVolt = str2double(extractAfter(parts{i}, 'V1='));
                        if isnan(obj.lastVolt)
                            obj.lastVolt = 0;
                        end
                    elseif startsWith(parts{i}, 'V2=')
                        obj.lastVolt2 = str2double(extractAfter(parts{i}, 'V2='));
                        if isnan(obj.lastVolt2)
                            obj.lastVolt2 = 0;
                        end
                    elseif startsWith(parts{i}, 'V3=')
                        obj.lastVolt3 = str2double(extractAfter(parts{i}, 'V3='));
                        if isnan(obj.lastVolt3)
                            obj.lastVolt3 = 0;
                        end
                    end
                end
                
                % Debug: mostrar valores recibidos
                fprintf('V1=%.2f, V2=%.2f, V3=%.2f\n', obj.lastVolt, obj.lastVolt2, obj.lastVolt3);
            catch e
                disp(['Error procesando dato: ' line]);
                disp(e.message);
            end
        end
        
        function closeFigure(obj)
            % Detener y eliminar timers
            if ~isempty(obj.updateTimer) && isa(obj.updateTimer, 'timer')
                if strcmp(obj.updateTimer.Running, 'on')
                    stop(obj.updateTimer);
                end
                delete(obj.updateTimer);
            end
            
            % Cerrar puerto serial
            if ~isempty(obj.serialObj) && isvalid(obj.serialObj)
                delete(obj.serialObj);
            end
            
            % Cerrar figura
            delete(obj.fig);
            disp('Monitor finalizado');
        end
    end
end