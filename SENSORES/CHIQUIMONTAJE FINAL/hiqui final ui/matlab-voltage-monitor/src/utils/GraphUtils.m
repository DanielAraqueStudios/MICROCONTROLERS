classdef GraphUtils
    methods (Static)
        function plotVoltage(ax, timePoints, voltPoints)
            % Plot voltage values over time
            plot(ax, timePoints, voltPoints, 'LineWidth', 2, 'Color', [0 0.4470 0.7410]);
            xlabel(ax, 'Time (s)');
            ylabel(ax, 'Voltage (V)');
            title(ax, 'Voltage Fluctuations Over Time');
            grid(ax, 'on');
            ylim(ax, [0 3.3]); % Assuming voltage range from 0 to 3.3V
            xlim(ax, [min(timePoints) max(timePoints)]);
        end
        
        function adjustAxesLimits(ax, timePoints, scrollMargin)
            % Adjust axes limits based on the current time points
            if length(timePoints) > scrollMargin
                xlim(ax, [max(1, timePoints(end) - scrollMargin), timePoints(end)]);
            else
                xlim(ax, [1, scrollMargin]);
            end
        end
        
        function setGraphStyle(ax)
            % Set the style for the graph
            ax.XColor = 'w'; % White X-axis color for dark mode
            ax.YColor = 'w'; % White Y-axis color for dark mode
            ax.FontSize = 12;
        end
    end
end