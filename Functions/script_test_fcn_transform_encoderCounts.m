%% Clean Up
clc
clear
close all

%% Revision History
%7/3/2023 -- Mariam Abdellatief
% -- Wrote code originally

%% Assertions / Testing outputs
wheel_radius = 0.3;
initial_counts_rear_left = 9;
initial_counts_rear_right = 11;
counts_per_revolution = 2;

%% Case: Two input velocities
wheel_velocity_rear_left = [11,2];
wheel_velocity_rear_right = [9,7];
delta_time = 5;
[discrete_encoder_count_rear_left, discrete_encoder_count_rear_right] = fcn_transform_encoderCounts(wheel_velocity_rear_left,wheel_velocity_rear_right,wheel_radius,initial_counts_rear_left,initial_counts_rear_right,delta_time,counts_per_revolution); %#ok<ASGLU>
expected_values.encoder_rear_left = [2312,2730]; %-->>IMP QUESTION: assertion fails bec the calculation takes the
% previous encoder value without floor, but by hand I used floored value
% --> Which is correct??
expected_values.encoder_rear_right = [1895,3361];
assert(isequal(discrete_encoder_count_rear_left,expected_values.encoder_rear_left));
assert(isequal(discrete_encoder_count_rear_right,expected_values.encoder_rear_right));

%% Case: Test with sine wave input 
% Parameters
amplitude = 1;      % Amplitude of the sine wave
frequency = 2;      % Frequency of the sine wave in Hz
samplingRate = 50; % Number of samples per second
duration = 1;       % Duration of the sine wave in seconds
% Generate time vector
t = linspace(0, duration, duration * samplingRate);
% Generate sine wave
sineWave = amplitude * sin(2 * pi * frequency * t);
wheel_velocity_rear_left = sineWave;
wheel_velocity_rear_right = sineWave; 
delta_time = samplingRate;
[discrete_encoder_count_rear_left, discrete_encoder_count_rear_right] = fcn_transform_encoderCounts(wheel_velocity_rear_left,wheel_velocity_rear_right,wheel_radius,initial_counts_rear_left,initial_counts_rear_right,delta_time,counts_per_revolution);
plot(t,sineWave)
hold on
plot(t,discrete_encoder_count_rear_left)
hold off
legend("Sine Wave Velocities", "Rear Left Encoder Counts")

%% Error throws
if 1 == 0
    %% Case: Incorrect number of inputs
    [discrete_encoder_count_rear_left, discrete_encoder_count_rear_right] = fcn_transform_encoderCounts(wheel_velocity_rear_right,wheel_radius,initial_counts_rear_left,initial_counts_rear_right,delta_time,counts_per_revolution); %#ok<ASGLU>
    %% Case: Initial counts are negative
    initial_counts_rear_left = -4; 
    [discrete_encoder_count_rear_left, discrete_encoder_count_rear_right] = fcn_transform_encoderCounts(wheel_velocity_rear_left,wheel_velocity_rear_right,wheel_radius,initial_counts_rear_left,initial_counts_rear_right,delta_time,counts_per_revolution); %#ok<ASGLU>
    %% Case: Delta time is non-positive
    delta_time = 0; 
    [discrete_encoder_count_rear_left, discrete_encoder_count_rear_right] = fcn_transform_encoderCounts(wheel_velocity_rear_left,wheel_velocity_rear_right,wheel_radius,initial_counts_rear_left,initial_counts_rear_right,delta_time,counts_per_revolution); %#ok<ASGLU>
    %% Case: Counts per revolution is non-positive
    counts_per_revolution = -6; 
    [discrete_encoder_count_rear_left, discrete_encoder_count_rear_right] = fcn_transform_encoderCounts(wheel_velocity_rear_left,wheel_velocity_rear_right,wheel_radius,initial_counts_rear_left,initial_counts_rear_right,delta_time,counts_per_revolution);
end