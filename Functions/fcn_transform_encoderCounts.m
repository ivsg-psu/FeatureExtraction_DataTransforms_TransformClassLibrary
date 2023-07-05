function [discrete_encoder_count_rear_left, discrete_encoder_count_rear_right] = fcn_transform_encoderCounts(wheel_velocity_rear_left,wheel_velocity_rear_right,wheel_radius,initial_counts_rear_left,initial_counts_rear_right,delta_time,counts_per_revolution)
% fcn_transform_encoderCounts
% This function calculates the number of encoder coutns for eahc wheel
% encoder. 
%
% FORMAT:
%
%       [discrete_encoder_count_rear_left, discrete_encoder_count_rear_right] = fcn_transform_encoderCounts(wheel_velocity_rear_left,wheel_velocity_rear_right,wheel_radius,initial_counts_rear_left,initial_counts_rear_right,delta_time,counts_per_revolution)
%
% INPUTS:
%       wheel_velocity_rear_left = An array of rear left wheel velocities in m/s.
%                                  Expected input is [v_1,v_2,v_3,...],
%                                  where each velocity corresponds to a
%                                  time step.
%
%       wheel_velocity_rear_right = An array of rear right wheel velocities in m/s.
%                                  Expected input is [v_1,v_2,v_3,...],
%                                  where each velocity corresponds to a
%                                  time step.
%
%       wheel_radius = Radius of wheel in meters.
%
%       initial_counts_rear_left = Initial rear left encoder counts before data
%                                  collection started. [counts]
%
%       initial_counts_rear_right = Initial rear left encoder counts before data
%                                  collection started. [counts]
%
%       delta_time = Encoder time step in seconds.
%
%       counts_per_revolution = Encoder default number of counts per
%                               revolution. [counts/rev.]
%      
% OUTPUTS:
%      
%       discrete_encoder_count_rear_left = An array of the calculated
%                                          discrete encoder counts for the
%                                          rear left wheel encoder. [counts]
%
%       discrete_encoder_count_rear_right = An array of the calculated
%                                          discrete encoder counts for the
%                                          rear right wheel encoder. [counts]
%       
% DEPENDENCIES:
%
%  No dependencies -----> Question: make it dependant on wheel velocity fcn?
%
% EXAMPLES:
%
%     See the script: script_test_fcn_transform_encoderCounts
%     for a full test suite.

% Revision history:
%     
% 2023_06_30: Mariam Abdellatief
% -- Wrote the code originally 
% 2023_07_03: Mariam Abdellatief
% -- Functionalized code 
% -- Added function definition 
% -- Wrote test script

%% Inputs
%Check number of inputs
if nargin < 7 || nargin > 7
        error ('Incorrect number of input arguments')
end 

if initial_counts_rear_left < 0 ||initial_counts_rear_right < 0
    error("Encoder initial counts can't be negative");
end

if delta_time < 0 || delta_time == 0
    error("Time step has to be a positive number");
end

if counts_per_revolution < 0 || counts_per_revolution == 0
    error("Encoder counts per revolution have to be positive");
end


%% Main 
wheel_w_rear_left = wheel_velocity_rear_left ./ wheel_radius;
continous_count_derivative_rear_left = counts_per_revolution * 2 * pi * wheel_w_rear_left;
continous_counts_rear_left = cumsum(continous_count_derivative_rear_left * delta_time) + initial_counts_rear_left;
discrete_encoder_count_rear_left = floor(continous_counts_rear_left);

wheel_w_rear_right = wheel_velocity_rear_right ./ wheel_radius;
continous_count_derivative_rear_right = counts_per_revolution * 2 * pi * wheel_w_rear_right;
continous_counts_rear_right = cumsum(continous_count_derivative_rear_right * delta_time) + initial_counts_rear_right;
discrete_encoder_count_rear_right = floor(continous_counts_rear_right);

end 