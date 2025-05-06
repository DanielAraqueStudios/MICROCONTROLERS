% Script to run the STM32 Voltage Monitor
clear;
clc;

% Configure port and baud rate
port = 'COM3'; % Change this to match your port
baudRate = 9600;

% Create and start the monitor
monitor = STM32VoltageMonitor(port, baudRate);
