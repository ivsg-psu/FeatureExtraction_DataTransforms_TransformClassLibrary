%% Clean Up
clc
clear 
close all 
% Created on 6/22/2023 by Mariam Abdellatief 

%% Define wheel position and chassis rotational velocity
% WARNING: Wheel position measurement is just an example, not the actual value
pos_rear_left = [0,10,0]; % Position from center of wheel in Vehicle-body coordinates
pos_rear_right = [0,-10,0]; % Position from center of wheel in Vehicle-body coordinates
chassis_w = [0,0,90]; 
chassis_alpha = [10,2,5];
v_vehicle = 10; 
vehicle_acceleration = 2;
tire_radius = 0.30; % There's a seperate function to estimate this value
 
%% Find wheel's linear velocity and rear encoder angular velocity
wheel_v_rear_left = v_vehicle + cross (chassis_w,pos_rear_left);
wheel_v_rear_right = v_vehicle + cross (chassis_w,pos_rear_right);
encoder_w_rear_left = wheel_v_rear_left(1) / tire_radius;
encoder_w_rear_right = wheel_v_rear_right(1) / tire_radius;

%% Check tire slide and vehicle bounce
tire_slide_rear_left = wheel_v_rear_left(2);
tire_slide_rear_right = wheel_v_rear_right(2);
vehicle_bounce_rear_left = wheel_v_rear_left(3);
vehicle_bounce_rear_right = wheel_v_rear_right(3);

%% Find IMU acceleration  
wheel_acceleration_rear_left = vehicle_acceleration + cross(chassis_alpha,pos_rear_left); 
wheel_acceleration_rear_right = vehicle_acceleration + cross(chassis_alpha,pos_rear_right);



