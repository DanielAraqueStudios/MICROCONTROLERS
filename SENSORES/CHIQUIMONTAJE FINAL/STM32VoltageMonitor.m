classdef STM32VoltageMonitor < handle
    properties
        % Configuración del puerto serial
        serialPort = 'COM21';  
        baudRate = 9600;     % Cambiado de 9600 a 115200 baudios
        
        % Configuración de la gráfica
        maxPoints = 100;  % Reducido para mejor visualización
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
        
        % BPM properties
        bpmText;           % Text display for BPM
        lastPeakTime = 0;  % Time of last peak
        peakThreshold = 2; % Voltage threshold for peak detection
        bpmValue = 0;      % Current BPM value
        peakTimes = [];    % Array to store peak times
        MAX_PEAK_WINDOW = 10; % Maximum number of peaks to consider
        
        % BPM detection properties
        windowSize = 10;         % Tamaño de la ventana para detección
        lastValues = [];         % Buffer circular para valores
        isPeakDetected = false;  % Estado de detección de pico
        minPeakDistance = 0.2;   % Tiempo mínimo entre picos (segundos)
        
        % Additional axes
        axRespiration;    % Axes for respiration pattern
        axSweat;          % Axes for sweat level
        respirationPlot;  % Plot line for respiration
        sweatBar;         % Bar plot for sweat level
        
        % Additional data
        lastVolt2 = 0;    % Respiration voltage
        lastVolt3 = 0;    % Sweat voltage
        timePoints2 = []; % For respiration
        voltPoints2 = []; % For respiration
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
                'Position', [0.15 0.57 0.75 0.35], ...  % Adjusted position
                'Color', [0.12 0.12 0.12], ...          % Slightly darker than background
                'XColor', [0.7 0.7 0.7], ...            % Softer grid color
                'YColor', [0.7 0.7 0.7], ...
                'GridColor', [0.2 0.2 0.2], ...         % Subtle grid
                'GridAlpha', 0.3);                      % Semi-transparent grid
            
            title(obj.ax, 'ECG', 'Color', [0.8 0.8 0.8], 'FontWeight', 'bold');
            xlabel(obj.ax, 'Tiempo (s)', 'Color', [0.7 0.7 0.7]);
            ylabel(obj.ax, 'Voltaje (V)', 'Color', [0.7 0.7 0.7]);
            grid(obj.ax, 'on');
            ylim(obj.ax, [0 3.3]);
            
            % Respiration Plot (middle)
            obj.axRespiration = axes('Parent', obj.fig, ...
                'Position', [0.15 0.12 0.75 0.35], ...  % Adjusted position
                'Color', [0.12 0.12 0.12], ...
                'XColor', [0.7 0.7 0.7], ...
                'YColor', [0.7 0.7 0.7], ...
                'GridColor', [0.2 0.2 0.2], ...
                'GridAlpha', 0.3);
            
            title(obj.axRespiration, 'Patrón de Respiración', 'Color', [0.8 0.8 0.8], 'FontWeight', 'bold');
            xlabel(obj.axRespiration, 'Tiempo (s)', 'Color', [0.7 0.7 0.7]);
            ylabel(obj.axRespiration, 'Voltaje (V)', 'Color', [0.7 0.7 0.7]);
            grid(obj.axRespiration, 'on');
            ylim(obj.axRespiration, [0 3.3]);
            
            % Sweat Level Bar (right) - Ajustada posición y tamaño
            obj.axSweat = axes('Parent', obj.fig, ...
                'Position', [0.92 0.2 0.05 0.6], ...   % Aumentado ancho de 0.035 a 0.05
                'Color', [0.12 0.12 0.12], ...
                'XColor', 'none', ...
                'YColor', [0.7 0.7 0.7], ...
                'Box', 'on', ...
                'GridColor', [0.3 0.3 0.3], ...
                'GridAlpha', 0.3, ...
                'TitleFontWeight', 'bold', ...           % Añadido peso de fuente para el título
                'TitleFontSizeMultiplier', 1.2);        % Aumentar tamaño relativo del título

            title(obj.axSweat, 'SUDORACIÓN', ...
                'Color', [1 1 1], ...
                'FontWeight', 'bold', ...
                'FontSize', 11, ...                      % Aumentado tamaño de fuente
                'Units', 'normalized', ...               % Usar unidades normalizadas
                'Position', [0.5, 1.1, 0]);             % Ajustar posición vertical del título
            
            grid(obj.axSweat, 'on');
            ylim(obj.axSweat, [0 100]);
            ylabel(obj.axSweat, '%', 'Color', [0.7 0.7 0.7]);

            % Initialize plots with modern style
            obj.linePlot = line(obj.ax, NaN, NaN, ...
                'Color', [0.3 0.8 1], ...               % Brighter blue
                'LineWidth', 1.5);
            
            obj.respirationPlot = line(obj.axRespiration, NaN, NaN, ...
                'Color', [0.3 1 0.5], ...               % Brighter green
                'LineWidth', 1.5);
            
            obj.sweatBar = bar(obj.axSweat, 0, ...
                'FaceColor', [1 0.3 0.3], ...           % Brighter red
                'EdgeColor', 'none');                   % Remove edge for cleaner look
            
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
            obj.bpmText = uicontrol('Parent', bpmPanel, ...
                'Style', 'text', ...
                'String', '0 BPM', ...
                'Position', [10 10 120 40], ...
                'BackgroundColor', [0.12 0.12 0.12], ...
                'ForegroundColor', [0 1 0], ...
                'FontSize', 20, ...
                'FontWeight', 'bold');
            
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
            % Leer datos disponibles
            while obj.serialObj.NumBytesAvailable > 0
                line = readline(obj.serialObj);
                obj.parseData(line);
            end
            
            if obj.lastVolt ~= 0
                obj.sampleCount = obj.sampleCount + 1;
                currentTime = now * 24 * 3600; % Tiempo actual en segundos
                
                % Actualizar buffer circular
                obj.lastValues(end+1) = obj.lastVolt;
                if length(obj.lastValues) > obj.windowSize
                    obj.lastValues = obj.lastValues(end-obj.windowSize+1:end);
                end
                
                % Detección de picos usando ventana deslizante
                if length(obj.lastValues) >= 3
                    middleIndex = ceil(length(obj.lastValues)/2);
                    middleValue = obj.lastValues(middleIndex);
                    
                    % Verificar si es un pico local
                    isPeak = middleValue > mean(obj.lastValues) && ...
                            all(middleValue >= obj.lastValues(1:middleIndex-1)) && ...
                            all(middleValue >= obj.lastValues(middleIndex+1:end));
                    
                    if isPeak && ~obj.isPeakDetected && ...
                       (currentTime - obj.lastPeakTime) > obj.minPeakDistance
                        % Pico detectado
                        obj.peakTimes(end+1) = currentTime;
                        obj.lastPeakTime = currentTime;
                        obj.isPeakDetected = true;
                        
                        % Mantener solo los últimos 10 picos
                        if length(obj.peakTimes) > 10
                            obj.peakTimes = obj.peakTimes(end-9:end);
                        end
                        
                        % Calcular BPM basado en intervalos entre picos
                        if length(obj.peakTimes) >= 2
                            intervals = diff(obj.peakTimes);
                            avgInterval = mean(intervals);
                            obj.bpmValue = round(60 / avgInterval);
                            
                            % Actualizar display de BPM
                            set(obj.bpmText, 'String', sprintf('%d BPM', obj.bpmValue));
                            
                            % Actualizar color según rango de BPM
                            if obj.bpmValue < 60
                                set(obj.bpmText, 'ForegroundColor', [1 1 0]); % Amarillo
                            elseif obj.bpmValue > 100
                                set(obj.bpmText, 'ForegroundColor', [1 0 0]); % Rojo
                            else
                                set(obj.bpmText, 'ForegroundColor', [0 1 0]); % Verde
                            end
                        end
                    elseif ~isPeak
                        obj.isPeakDetected = false;
                    end
                end
                
                % Actualizar puntos de la gráfica con el voltaje real
                obj.timePoints(end+1) = obj.sampleCount;
                obj.voltPoints(end+1) = obj.lastVolt;
                
                % Update Respiration plot
                obj.timePoints2(end+1) = obj.sampleCount;
                obj.voltPoints2(end+1) = obj.lastVolt2;
                
                % Mantener el tamaño máximo del buffer
                if length(obj.timePoints) > obj.maxPoints + obj.scrollMargin
                    obj.timePoints = obj.timePoints(end-obj.maxPoints-obj.scrollMargin+1:end);
                    obj.voltPoints = obj.voltPoints(end-obj.maxPoints-obj.scrollMargin+1:end);
                end
                
                if length(obj.timePoints2) > obj.maxPoints + obj.scrollMargin
                    obj.timePoints2 = obj.timePoints2(end-obj.maxPoints-obj.scrollMargin+1:end);
                    obj.voltPoints2 = obj.voltPoints2(end-obj.maxPoints-obj.scrollMargin+1:end);
                end
                
                % Actualizar línea de la gráfica
                set(obj.linePlot, 'XData', obj.timePoints, 'YData', obj.voltPoints);
                set(obj.respirationPlot, 'XData', obj.timePoints2, 'YData', obj.voltPoints2);
                
                % Update Sweat Level bar
                sweatPercentage = (obj.lastVolt3 / 3.3) * 100;  % Convert to percentage
                set(obj.sweatBar, 'YData', sweatPercentage);
                
                % Actualizar leyenda con el voltaje actual
                legend(obj.ax, sprintf('Voltaje Actual: %.2fV', obj.lastVolt), ...
                    'TextColor', [0.8 0.8 0.8], ...
                    'Color', [0.2 0.2 0.2]);
                
                % Ajustar límites si es necesario
                if obj.sampleCount > obj.startScroll
                    xlim(obj.ax, [max(1, obj.sampleCount - obj.maxPoints), obj.sampleCount]);
                    xlim(obj.axRespiration, [max(1, obj.sampleCount - obj.maxPoints), obj.sampleCount]);
                else
                    xlim(obj.ax, [1, obj.maxPoints]);
                end
                
                % Forzar actualización de la interfaz
                drawnow;
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