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
        
        % Umbrales de respiración
        VOLT_MIN = 1.0;  % Voltaje mínimo esperado
        VOLT_MAX = 1.3;  % Voltaje máximo esperado
        lastBreathState = 0;
        
        % Conteo de respiraciones
        breathCount = 0;
        lastBreathTime = 0;
        
        % UI Elements
        warningText;  % Texto de advertencia en GUI
        warningImage; % Imagen de advertencia
        imageAxes;    % Axes para la imagen
        voltageText;  % Texto para mostrar el voltaje actual
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
                'Position', [0.3 0.1 0.65 0.8]);
            title(obj.ax, 'Patrón de Respiración');
            xlabel(obj.ax, 'Tiempo (s)');
            ylabel(obj.ax, 'Estado');
            grid(obj.ax, 'on');
            ylim(obj.ax, [0.9 1.4]);  % Rango del voltaje con márgenes
            % Cambiar etiquetas del eje Y
            yticks([1.0 1.3]);  % Solo mostrar dos niveles
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
            
            % Agregar texto para mostrar voltaje (después del warningText)
            obj.voltageText = uicontrol('Style', 'text', ...
                'Position', [50 500 200 30], ...
                'String', 'Voltaje: 0.00 V', ...
                'BackgroundColor', 'white', ...
                'FontSize', 12, ...
                'Visible', 'on');
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
                
                % Actualizar puntos de la gráfica con el voltaje real invertido
                obj.timePoints(end+1) = obj.sampleCount;
                obj.voltPoints(end+1) = -1 * obj.lastVolt + 2.3; % Invertir y ajustar offset
                
                % Detectar inhalación/exhalación basado en tendencia
                if length(obj.voltPoints) > 1
                    voltDiff = obj.voltPoints(end) - obj.voltPoints(end-1);
                    if abs(voltDiff) > 0.01  % Umbral de cambio significativo
                        if voltDiff > 0
                            estado = 'Inhalando';
                        else
                            estado = 'Exhalando';
                        end
                        
                        % Contar respiraciones rápidas
                        if voltDiff * (obj.lastBreathState) < 0  % Cambio de dirección
                            currentTime = now * 24 * 3600;
                            if currentTime - obj.lastBreathTime <= 1
                                obj.breathCount = obj.breathCount + 1;
                            else
                                obj.breathCount = 1;
                            end
                            obj.lastBreathTime = currentTime;
                            
                            % Mostrar advertencia si respira muy rápido
                            if obj.breathCount > 5
                                set(obj.warningText, 'Visible', 'on');
                                obj.imageAxes.Visible = 'on';
                            else
                                set(obj.warningText, 'Visible', 'off');
                                obj.imageAxes.Visible = 'off';
                            end
                        end
                        obj.lastBreathState = sign(voltDiff);
                    else
                        estado = 'Neutral';
                    end
                else
                    estado = 'Iniciando';
                end
                
                % Mantener el tamaño máximo del buffer
                if length(obj.timePoints) > obj.maxPoints + obj.scrollMargin
                    obj.timePoints = obj.timePoints(end-obj.maxPoints-obj.scrollMargin+1:end);
                    obj.voltPoints = obj.voltPoints(end-obj.maxPoints-obj.scrollMargin+1:end);
                end
                
                % Actualizar línea de la gráfica
                set(obj.linePlot, 'XData', obj.timePoints, 'YData', obj.voltPoints);
                
                % Actualizar leyenda - solo mostrar estado sin voltaje
                legend(obj.ax, sprintf('Estado: %s', estado));
                
                % Actualizar display de voltaje
                set(obj.voltageText, 'String', sprintf('Voltaje: %.2f V', obj.lastVolt));
                
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