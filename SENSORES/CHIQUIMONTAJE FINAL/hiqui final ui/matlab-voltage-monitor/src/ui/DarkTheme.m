classdef DarkTheme
    methods (Static)
        function applyTheme(fig)
            % Set the figure background color
            set(fig, 'Color', [0.1 0.1 0.1]);
            
            % Set the axes properties
            ax = gca;
            set(ax, 'Color', [0.2 0.2 0.2], ...
                     'XColor', [1 1 1], ...
                     'YColor', [1 1 1], ...
                     'GridColor', [0.5 0.5 0.5]);
                 
            % Set title and labels
            title(ax, 'Voltage Fluctuations', 'Color', [1 1 1]);
            xlabel(ax, 'Time (s)', 'Color', [1 1 1]);
            ylabel(ax, 'Voltage (V)', 'Color', [1 1 1]);
            
            % Set legend properties
            legend('show', 'TextColor', [1 1 1], 'Location', 'best');
            
            % Set UI control properties
            uicontrols = findall(fig, 'Type', 'uicontrol');
            for i = 1:length(uicontrols)
                set(uicontrols(i), 'BackgroundColor', [0.3 0.3 0.3], ...
                                   'ForegroundColor', [1 1 1]);
            end
        end
    end
end