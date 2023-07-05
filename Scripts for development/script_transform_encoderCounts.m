%% Clean Up
clc
clear 
close all 
% Created on 6/30/2023 by Mariam Abdellatief

%% Inputs
wheel_velocity = [11,2,3];
wheel_radius = 0.3;
initial_counts = 9;
delta_time = 5;
total_time = 10;
counts_per_revolution = 21;

%% Calculating output
% Loop Method 
%previous_continous_count = initial_counts;
%continous_counts = [0];
% for i = 1:3
%     wheel_w(i) = wheel_velocity(i) / wheel_radius;
%     continous_count_derivative(i) = counts_per_revolution*2*pi*wheel_w(i);
%     continous_counts(i)= continous_count_derivative(i) * delta_time + previous_continous_count;
%     previous_continous_count = continous_counts(i);
%     discrete_encoder_count(i) = floor (continous_counts(i));
% end 

% Vectorized Method 
wheel_w = wheel_velocity ./ wheel_radius;
continous_count_derivative = counts_per_revolution * 2 * pi * wheel_w;
previous_continous_count = initial_counts;
continous_counts = cumsum(continous_count_derivative * delta_time) + previous_continous_count;
discrete_encoder_count = floor(continous_counts);





