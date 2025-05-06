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
            obj.fig = figure('Name', 'Biofeedback', ...
                'NumberTitle', 'off', ...
                'Position', [100 100 1000 600], ...
                'CloseRequestFcn', @(src,event)obj.closeFigure());
            
            % Área de gráfica
            obj.ax = axes('Parent', obj.fig, ...
                'Position', [0.3 0.1 0.65 0.8]);
            title(obj.ax, 'Lectura de Voltaje en Tiempo Real');
            xlabel(obj.ax, 'Número de Muestra');
            ylabel(obj.ax, 'Voltaje (V)');
            grid(obj.ax, 'on');
            ylim(obj.ax, [0 3.5]);
            
            % Línea inicial
            obj.linePlot = line(obj.ax, NaN, NaN, ...
                'Color', 'b', ...
                'LineWidth', 2);
            
            % Controles
            uicontrol('Style', 'text', ...
                'Position', [50 500 150 20], ...
                'String', 'Tiempo de muestreo (s):', ...
                'HorizontalAlignment', 'left');
            
            timerEdit = uicontrol('Style', 'edit', ...
                'Position', [50 480 150 20], ...
                'String', '0.1', ...
                'Callback', @(src,~)obj.setTimer(src.String));
            
            uicontrol('Style', 'pushbutton', ...
                'Position', [50 450 150 25], ...
                'String', 'Configurar Muestreo', ...
                'Callback', @(src,event)obj.setTimer(timerEdit.String));
            
            uicontrol('Style', 'text', ...
                'Position', [50 400 150 20], ...
                'String', 'Número de muestras:', ...
                'HorizontalAlignment', 'left');
            
            samplesEdit = uicontrol('Style', 'edit', ...
                'Position', [50 380 150 20], ...
                'String', '10', ...
                'Callback', @(src,~)obj.setSamples(src.String));
            
            uicontrol('Style', 'pushbutton', ...
                'Position', [50 350 150 25], ...
                'String', 'Configurar Muestras', ...
                'Callback', @(src,event)obj.setSamples(samplesEdit.String));
            
            % Información
            uicontrol('Style', 'text', ...
                'Position', [50 250 200 80], ...
                'String', sprintf(['Configuración:\n', ...
                                  '- Timer: ARR fijo = 15999\n', ...
                                  '- PSC calculado automáticamente\n', ...
                                  '- Muestras: enteros > 0']), ...
                'HorizontalAlignment', 'left');
        end
        
        function psc = calculatePSC(obj, timeSeconds)
            try
                timeSeconds = str2double(timeSeconds);
                if timeSeconds <= 0
                    psc = 0;
                    return;
                end
                psc = (obj.CLOCK_FREQ / ((obj.DEFAULT_ARR + 1) * (1/timeSeconds))) - 1;
                psc = ceil(psc);
            catch
                psc = 0;
            end
        end
        
        function setTimer(obj, timeSeconds)
            psc = obj.calculatePSC(timeSeconds);
            if psc > 0
                cmd = sprintf('P%d,%d\n', obj.DEFAULT_ARR, psc);
                obj.sendCommand(cmd);
                disp(['Configurado tiempo de muestreo: ' cmd]);
            else
                errordlg('Tiempo de muestreo no válido', 'Error');
            end
        end
        
        function setSamples(obj, samples)
            try
                m = str2double(samples);
                if m > 0
                    cmd = sprintf('M%d\n', m);
                    obj.sendCommand(cmd);
                    disp(['Configurado número de muestras: ' cmd]);
                else
                    errordlg('Número de muestras debe ser > 0', 'Error');
                end
            catch
                errordlg('Número de muestras no válido', 'Error');
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
            
            % Actualizar gráfica si hay nuevos datos
            if obj.lastVolt ~= 0
                obj.sampleCount = obj.sampleCount + 1;
                obj.timePoints(end+1) = obj.sampleCount;
                obj.voltPoints(end+1) = obj.lastVolt;
                
                % Mantener el tamaño máximo del buffer
                if length(obj.timePoints) > obj.maxPoints + obj.scrollMargin
                    obj.timePoints = obj.timePoints(end-obj.maxPoints-obj.scrollMargin+1:end);
                    obj.voltPoints = obj.voltPoints(end-obj.maxPoints-obj.scrollMargin+1:end);
                end
                
                % Actualizar línea de la gráfica
                set(obj.linePlot, 'XData', obj.timePoints, 'YData', obj.voltPoints);
                
                % Actualizar leyenda
                legend(obj.ax, sprintf('Voltaje: %.3f V', obj.lastVolt));
                
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