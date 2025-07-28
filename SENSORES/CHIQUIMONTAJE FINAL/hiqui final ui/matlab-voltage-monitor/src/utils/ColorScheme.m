classdef ColorScheme
    properties (Constant)
        BackgroundColor = [0.1, 0.1, 0.1]; % Dark background
        ForegroundColor = [1, 1, 1]; % White text
        GridColor = [0.5, 0.5, 0.5]; % Gray grid
        LineColor = [0, 0.7, 1]; % Blue line for graphs
        WarningColor = [1, 0, 0]; % Red for warnings
        InhaleColor = [0, 1, 0]; % Green for inhalation
        ExhaleColor = [1, 0.5, 0]; % Orange for exhalation
    end
    
    methods (Static)
        function applyDarkTheme(fig)
            % Apply dark theme colors to the figure
            set(fig, 'Color', ColorScheme.BackgroundColor);
            ax = findall(fig, 'Type', 'axes');
            set(ax, 'Color', ColorScheme.BackgroundColor, ...
                     'XColor', ColorScheme.ForegroundColor, ...
                     'YColor', ColorScheme.ForegroundColor, ...
                     'GridColor', ColorScheme.GridColor);
            title(ax, 'Voltage Monitor', 'Color', ColorScheme.ForegroundColor);
            xlabel(ax, 'Time (s)', 'Color', ColorScheme.ForegroundColor);
            ylabel(ax, 'Voltage (V)', 'Color', ColorScheme.ForegroundColor);
        end
    end
end