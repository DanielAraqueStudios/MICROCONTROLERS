# MATLAB Voltage Monitor

## Overview
The MATLAB Voltage Monitor is a project designed to visualize voltage fluctuations over time using a modern dark mode user interface. This application is particularly useful for monitoring voltage values in real-time, providing an intuitive and visually appealing way to analyze data.

## Project Structure
```
matlab-voltage-monitor
├── src
│   ├── VoltageMonitor.m
│   ├── utils
│   │   ├── ColorScheme.m
│   │   └── GraphUtils.m
│   └── ui
│       └── DarkTheme.m
├── tests
│   └── VoltageMonitorTest.m
├── assets
│   └── styles.mat
└── README.md
```

## Files Description

- **src/VoltageMonitor.m**: Defines the `VoltageMonitor` class responsible for monitoring voltage values. It includes methods for initializing the UI, updating the graph with voltage data, and handling serial communication.

- **src/utils/ColorScheme.m**: Contains utility functions for defining and applying a dark color scheme for the UI components, enhancing the visual appeal and user experience.

- **src/utils/GraphUtils.m**: Provides utility functions for graphing, including methods to plot voltage values over time and adjust graph settings such as axes limits and labels.

- **src/ui/DarkTheme.m**: Sets up the dark mode theme for the user interface, including the layout and styling of UI elements to ensure a modern look and feel.

- **tests/VoltageMonitorTest.m**: Contains unit tests for the `VoltageMonitor` class, ensuring that the functionality works as expected and that the UI updates correctly with incoming voltage data.

- **assets/styles.mat**: Stores style configurations and settings that can be loaded into the application to maintain consistency in the UI design.

## Setup Instructions
1. Clone the repository or download the project files.
2. Open MATLAB and navigate to the project directory.
3. Load the styles using the `load('assets/styles.mat')` command.
4. Run the `VoltageMonitor` class to start the application.

## Usage
Once the application is running, it will connect to the specified serial port and begin monitoring voltage values. The graph will update in real-time, displaying the fluctuations of voltage over time. The dark mode theme enhances visibility and reduces eye strain during prolonged use.

## Features
- Real-time voltage monitoring
- Modern dark mode user interface
- Intuitive graphing of voltage fluctuations
- Utility functions for color schemes and graph settings
- Unit tests to ensure functionality and reliability

## License
This project is licensed under the MIT License. See the LICENSE file for more details.