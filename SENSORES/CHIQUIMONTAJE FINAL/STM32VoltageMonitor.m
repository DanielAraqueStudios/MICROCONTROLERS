classdef STM32VoltageMonitor < handle
    properties
        % Configuración del puerto serial
        serialPort = 'COM15';
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
        
        % Rangos de voltaje para respiración
        VOLT_RANGE_MIN = 0.0;    % Voltaje mínimo absoluto
        VOLT_RANGE_MAX = 3.3;    % Voltaje máximo absoluto
        VOLT_THRESHOLD = 1.15;   % Umbral entre inhalación y exhalación
        GRAPH_MIN = -2.0;        % Límite inferior de la gráfica
        GRAPH_MAX = 0;           % Límite superior de la gráfica
        
        % UI Elements
        warningText;  % Texto de advertencia en GUI
        warningImage; % Imagen de advertencia
        imageAxes;    % Axes para la imagen
        
        % UI Elements para animación de respiración
        circleAx;      % Axes para el círculo de respiración
        circleHandle;  % Handle para el círculo
        baseRadius = 50; % Radio base del círculo
        maxScale = 1.5; % Escala máxima de expansión
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
            % Crear figura y controles
            obj.fig = figure('Name', 'Monitor de Respiración', ...
                'NumberTitle', 'off', ...
                'Position', [100 100 1000 600], ...
                'CloseRequestFcn', @(src,event)obj.closeFigure());
            
            % Área de gráfica
            obj.ax = axes('Parent', obj.fig, ...
                'Position', [0.3 0.4 0.65 0.55]);
            title(obj.ax, 'Patrón de Respiración');
            xlabel(obj.ax, 'Tiempo (s)');
            ylabel(obj.ax, 'Estado');
            grid(obj.ax, 'on');
            ylim(obj.ax, [obj.GRAPH_MIN obj.GRAPH_MAX]);  % Zoom ajustado
            yticks([-1.5 -0.5]);  % Marcas en Y ajustadas
            yticklabels({'Exhalación', 'Inhalación'});
            
            % Línea inicial
            obj.linePlot = line(obj.ax, NaN, NaN, ...
                'Color', 'b', ...
                'LineWidth', 2);
            
            % Agregar texto de advertencia (inicialmente invisible)
            obj.warningText = uicontrol('Style', 'text', ...
                'Position', [50 550 200 30], ...
                'String', '¡Cálmate parce, respira más despacio!', ...
                'BackgroundColor', 'red', ...
                'ForegroundColor', 'white', ...
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
            
            % Crear axes para el círculo de respiración
            obj.circleAx = axes('Parent', obj.fig, ...
                'Position', [0.4 0.05 0.45 0.25], ... % Debajo de la gráfica principal
                'XLim', [-100 100], ...
                'YLim', [-100 100]);
            axis(obj.circleAx, 'equal');
            axis(obj.circleAx, 'off');
            
            % Crear círculo inicial
            theta = linspace(0, 2*pi, 100);
            x = obj.baseRadius * cos(theta);
            y = obj.baseRadius * sin(theta);
            obj.circleHandle = fill(obj.circleAx, x, y, 'b', ...
                'FaceAlpha', 0.3, ...
                'EdgeColor', [0.2 0.6 1], ...
                'LineWidth', 2);
            
            % Añadir texto guía
            text(obj.circleAx, 0, -80, 'Respira con el círculo', ...
                'HorizontalAlignment', 'center', ...
                'FontSize', 12, ...
                'Color', [0.2 0.6 1]);
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
                
                % Determinar estado de respiración basado en el umbral
                if obj.lastVolt < obj.VOLT_THRESHOLD
                    estado = 'Inhalando';
                else
                    estado = 'Exhalando';
                end
                
                % Actualizar puntos de la gráfica con el voltaje real invertido y escalado
                obj.timePoints(end+1) = obj.sampleCount;
                scaledVolt = -1 * (obj.lastVolt - obj.VOLT_THRESHOLD) - 0.5;  % Centrar y escalar
                obj.voltPoints(end+1) = scaledVolt;
                
                % Mantener el tamaño máximo del buffer
                if length(obj.timePoints) > obj.maxPoints + obj.scrollMargin
                    obj.timePoints = obj.timePoints(end-obj.maxPoints-obj.scrollMargin+1:end);
                    obj.voltPoints = obj.voltPoints(end-obj.maxPoints-obj.scrollMargin+1:end);
                end
                
                % Actualizar línea de la gráfica
                set(obj.linePlot, 'XData', obj.timePoints, 'YData', obj.voltPoints);
                
                % Actualizar estado de respiración
                if obj.lastVolt < obj.VOLT_THRESHOLD
                    estado = 'Inhalando';
                else
                    estado = 'Exhalando';
                end
                
                % Actualizar leyenda
                legend(obj.ax, sprintf('Estado: %s (%.2fV)', estado, obj.lastVolt));
                
                % Ajustar límites si es necesario
                if obj.sampleCount > obj.startScroll
                    xlim(obj.ax, [max(1, obj.sampleCount - obj.maxPoints + obj.scrollMargin), ...
                                 obj.sampleCount + obj.scrollMargin]);
                else
                    xlim(obj.ax, [1, obj.maxPoints]);
                end
                
                % Actualizar animación del círculo
                scale = 1;
                if estado == 'Inhalando'
                    % Expandir círculo basado en el voltaje
                    scale = 1 + (obj.VOLT_THRESHOLD - obj.lastVolt) / ...
                           obj.VOLT_THRESHOLD * (obj.maxScale - 1);
                else
                    % Contraer círculo
                    scale = 1 + (obj.lastVolt - obj.VOLT_THRESHOLD) / ...
                           (obj.VOLT_RANGE_MAX - obj.VOLT_THRESHOLD) * (1 - 1/obj.maxScale);
                end
                
                % Actualizar tamaño del círculo
                theta = linspace(0, 2*pi, 100);
                x = obj.baseRadius * scale * cos(theta);
                y = obj.baseRadius * scale * sin(theta);
                set(obj.circleHandle, 'XData', x, 'YData', y);
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