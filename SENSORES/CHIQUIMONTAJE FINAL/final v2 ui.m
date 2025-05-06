classdef STM32VoltageMonitor < handle
    properties
        serialPort % Puerto serial para la comunicación
        baudRate % Tasa de baudios para la comunicación
        serialObj % Objeto de puerto serial
        updateTimer % Temporizador para actualizaciones
        fig % Figura de la interfaz
    end
    
    methods
        function obj = STM32VoltageMonitor(port, rate)
            obj.serialPort = port;
            obj.baudRate = rate;
            obj.fig = figure('Name', 'STM32 Voltage Monitor', 'CloseRequestFcn', @(src, event)closeFigure(obj));
            obj.initializeSerial();
            obj.startTimer();
        end
        
        function initializeSerial(obj)
            try
                obj.serialObj = serialport(obj.serialPort, obj.baudRate);
                configureTerminator(obj.serialObj, "CR/LF");
                disp('Puerto serial inicializado correctamente');
            catch e
                error('Error al inicializar el puerto serial: %s', e.message);
            end
        end
        
        function startTimer(obj)
            obj.updateTimer = timer('ExecutionMode', 'fixedRate', 'Period', 1, ...
                'TimerFcn', @(~,~)obj.readVoltage());
            start(obj.updateTimer);
        end
        
        function readVoltage(obj)
            try
                % Leer datos del puerto serial
                data = readline(obj.serialObj);
                voltage = str2double(data);
                
                % Actualizar la interfaz gráfica
                if isvalid(obj.fig)
                    clf(obj.fig);
                    plot(obj.fig, voltage);
                    title(obj.fig, 'Voltaje en tiempo real');
                    xlabel(obj.fig, 'Tiempo (s)');
                    ylabel(obj.fig, 'Voltaje (V)');
                end
            catch e
                warning('Error al leer el voltaje: %s', e.message);
            end
        end
        
        function closeFigure(obj)
            try
                % Detener timer
                if ~isempty(obj.updateTimer) && isvalid(obj.updateTimer)
                    stop(obj.updateTimer);
                    delete(obj.updateTimer);
                end
                
                % Cerrar puerto serial
                if ~isempty(obj.serialObj) && isvalid(obj.serialObj)
                    delete(obj.serialObj);
                end
                
                % Eliminar la figura
                if isvalid(obj.fig)
                    delete(obj.fig);
                end
                
                disp('Monitor finalizado correctamente');
            catch e
                warning('Error durante el cierre: %s', e.message);
                % Forzar cierre de la figura en caso de error
                if isvalid(obj.fig)
                    delete(obj.fig);
                end
            end
        end
    end
end