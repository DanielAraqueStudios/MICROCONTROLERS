classdef STM32VoltageMonitor < handle
    properties
        % Configuración del puerto serial
        serialPort = 'COM21';  % Cambiado de COM15 a COM21
        baudRate = 9600;
        
        % Configuración de la gráfica
        maxPoints = 200;
        startScroll = 90;
        scrollMargin = 10;
        
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
                'Period', 0.05, ...
                'TimerFcn', @(src,event)obj.updatePlot());
            
            start(obj.updateTimer);
        end
        
        function createUI(obj)
            % Crear figura con tema oscuro
            obj.fig = figure('Name', 'ECG', ...
                'NumberTitle', 'off', ...
                'Position', [100 100 1000 600], ...
                'Color', [0.2 0.2 0.2], ...
                'CloseRequestFcn', @(src,event)obj.closeFigure());
            
            % Área de gráfica con tema oscuro
            obj.ax = axes('Parent', obj.fig, ...
                'Position', [0.3 0.1 0.65 0.8], ...
                'Color', [0.15 0.15 0.15], ...
                'XColor', [0.8 0.8 0.8], ...
                'YColor', [0.8 0.8 0.8], ...
                'GridColor', [0.3 0.3 0.3]);
            
            title(obj.ax, 'ECG', 'Color', [0.8 0.8 0.8]);
            xlabel(obj.ax, 'Tiempo (s)', 'Color', [0.8 0.8 0.8]);
            ylabel(obj.ax, 'Voltaje (V)', 'Color', [0.8 0.8 0.8]);
            grid(obj.ax, 'on');
            ylim(obj.ax, [obj.VOLTAGE_MIN obj.VOLTAGE_MAX]);
            
            % Línea inicial con estilo moderno
            obj.linePlot = line(obj.ax, NaN, NaN, ...
                'Color', [0.2 0.8 1], ...
                'LineWidth', 2);
            
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
            
            % Panel para BPM
            bpmPanel = uipanel('Parent', obj.fig, ...
                'Title', 'Ritmo Cardíaco', ...
                'Position', [0.05 0.8 0.2 0.15], ...
                'BackgroundColor', [0.2 0.2 0.2], ...
                'ForegroundColor', [0.8 0.8 0.8], ...
                'HighlightColor', [0.3 0.3 0.3]);
            
            % Display de BPM
            obj.bpmText = uicontrol('Parent', bpmPanel, ...
                'Style', 'text', ...
                'String', '0 BPM', ...
                'Position', [10 10 160 40], ...
                'BackgroundColor', [0.15 0.15 0.15], ...
                'ForegroundColor', [0 1 0], ...
                'FontSize', 20, ...
                'FontWeight', 'bold');
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
            
            % Actualizar gráfica si hay nuevos datos
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
                
                % Mantener el tamaño máximo del buffer
                if length(obj.timePoints) > obj.maxPoints + obj.scrollMargin
                    obj.timePoints = obj.timePoints(end-obj.maxPoints-obj.scrollMargin+1:end);
                    obj.voltPoints = obj.voltPoints(end-obj.maxPoints-obj.scrollMargin+1:end);
                end
                
                % Actualizar línea de la gráfica
                set(obj.linePlot, 'XData', obj.timePoints, 'YData', obj.voltPoints);
                
                % Actualizar leyenda con el voltaje actual
                legend(obj.ax, sprintf('Voltaje Actual: %.2fV', obj.lastVolt), ...
                    'TextColor', [0.8 0.8 0.8], ...
                    'Color', [0.2 0.2 0.2]);
                
                % Ajustar límites si es necesario
                if obj.sampleCount > obj.startScroll
                    xlim(obj.ax, [max(1, obj.sampleCount - obj.maxPoints + obj.scrollMargin), ...
                                 obj.sampleCount + obj.scrollMargin]);
                else
                    xlim(obj.ax, [1, obj.maxPoints]);
                end
            end
        end
        
        function parseData(obj, line)
            try
                line = strtrim(line);
                if startsWith(line, 'V1=')
                    obj.lastVolt = str2double(extractAfter(line, 'V1='));
                end
            catch
                disp(['Error procesando dato: ' line]);
            end
        end
        
        function closeFigure(obj)
            % Detener timer y cerrar puerto serial
            stop(obj.updateTimer);
            delete(obj.updateTimer);
            
            if ~isempty(obj.serialObj) && isvalid(obj.serialObj)
                delete(obj.serialObj);
            end
            
            % Cerrar figura
            delete(obj.fig);
            disp('Monitor finalizado');
        end
    end
end