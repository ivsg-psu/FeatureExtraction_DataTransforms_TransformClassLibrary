%% Clean Up
clc
clear
close all

%% Revision History
%6/27/2023 -- Mariam Abdellatief
% -- Wrote code originally
%7/6/2023 -- Mariam Abdellatief
% -- Added more test cases 

%% Assertions / Testing outputs

pos_rear_left = [0,2,0]; % Position from center of wheel in Vehicle-body coordinates
pos_rear_right = [0,-2,0]; % Position from center of wheel in Vehicle-body coordinates

%% Case: angular velocity of vehicle is positive in Z and zero in x and y
% (vehicle turning to the left), and linear velocity is zero.
% V_x should be negative for left enocder and positive for right encoder.
fprintf(1,'Testing simple case with only angular velocity in z. \n')
chassis_w = [0,0,9];
chassis_v = [0,0,0];
predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w,chassis_v); 
expected_values.wheel_v_rear_left = [-18 0 0];
expected_values.wheel_v_rear_right = [18 0 0];
assert(isequal(predicted_values.wheel_v_rear_left,expected_values.wheel_v_rear_left));
assert(isequal(predicted_values.wheel_v_rear_right,expected_values.wheel_v_rear_right));

%% Case: angular velocity of vehicle is negative in Z and zero in x and y
% (vehicle turning to the right), and linear velocity is zero.
% V_x should be positive for left enocder and negative for right encoder.
fprintf(1,'Testing simple case with only negative angular velocity in z. \n')
chassis_w = [0,0,-7];
predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w,chassis_v); 
expected_values.wheel_v_rear_left = [14 0 0];
expected_values.wheel_v_rear_right = [-14 0 0];
assert(isequal(predicted_values.wheel_v_rear_left,expected_values.wheel_v_rear_left));
assert(isequal(predicted_values.wheel_v_rear_right,expected_values.wheel_v_rear_right));

%% Case: Vehicle moves straight forward
fprintf(1,'Testing simple case with only linear velocity in x. \n')
chassis_w = [0,0,0];
chassis_v = [11,0,0];
predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w,chassis_v); 
expected_values.wheel_v_rear_left = [11 0 0];
expected_values.wheel_v_rear_right = [11 0 0];
assert(isequal(predicted_values.wheel_v_rear_left,expected_values.wheel_v_rear_left));
assert(isequal(predicted_values.wheel_v_rear_right,expected_values.wheel_v_rear_right));

%% Case: Vehicle moves straight backward
fprintf(1,'Testing simple case with only negative linear velocity in x. \n')
chassis_v = [-19,0,0];
predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w,chassis_v); 
expected_values.wheel_v_rear_left = [-19 0 0];
expected_values.wheel_v_rear_right = [-19 0 0];
assert(isequal(predicted_values.wheel_v_rear_left,expected_values.wheel_v_rear_left));
assert(isequal(predicted_values.wheel_v_rear_right,expected_values.wheel_v_rear_right));

%% Case: Vehicle turns right and moves forward while sliding to the right 
fprintf(1,'Testing complex case with negative sliding, linear velocity, and negative rotation in z. \n')
chassis_w = [0,0,-2];
chassis_v = [7,-3,0];
predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w,chassis_v); 
expected_values.wheel_v_rear_left = [11 -3 0];
expected_values.wheel_v_rear_right = [3 -3 0];
assert(isequal(predicted_values.wheel_v_rear_left,expected_values.wheel_v_rear_left));
assert(isequal(predicted_values.wheel_v_rear_right,expected_values.wheel_v_rear_right));

%% Case: Vehicle turns left and moves forward and sliding to the left
fprintf(1,'Testing complex case with positive sliding, linear velocity, and rotation. \n')
chassis_w = [0,0,13];
chassis_v = [5,11,0];
predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w,chassis_v); 
expected_values.wheel_v_rear_left = [-21 11 0];
expected_values.wheel_v_rear_right = [31 11 0];
assert(isequal(predicted_values.wheel_v_rear_left,expected_values.wheel_v_rear_left));
assert(isequal(predicted_values.wheel_v_rear_right,expected_values.wheel_v_rear_right));

%% Case: Vehicle turns right and moves backward while sliding to the right 
fprintf(1,'Testing complex case with negative sliding, negative linear velocity, and negative rotation. \n')
chassis_w = [0,0,-2];
chassis_v = [-7,-3,0];
predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w,chassis_v); 
expected_values.wheel_v_rear_left = [-3 -3 0];
expected_values.wheel_v_rear_right = [-11 -3 0];
assert(isequal(predicted_values.wheel_v_rear_left,expected_values.wheel_v_rear_left));
assert(isequal(predicted_values.wheel_v_rear_right,expected_values.wheel_v_rear_right));
 
%% Case: Vehicle turns left and moves backward while sliding to the left
fprintf(1,'Testing complex case with positive sliding, positive linear velocity, and positive rotation. \n')
chassis_w = [0,0,13];
chassis_v = [-5,11,0];
predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w,chassis_v); 
expected_values.wheel_v_rear_left = [-31 11 0];
expected_values.wheel_v_rear_right = [21 11 0];
assert(isequal(predicted_values.wheel_v_rear_left,expected_values.wheel_v_rear_left));
assert(isequal(predicted_values.wheel_v_rear_right,expected_values.wheel_v_rear_right));

%% Case: Vehicle turns left, moves backward, slides left, rolls, pitches, and bounces up 
% will give a warning because slide and bounce are large 
fprintf(1,'Testing complex case with positive sliding, negative linear velocity, positive rotation, positive bounce, positive roll, and positive pitch \n')
chassis_w = [2,3,13];
chassis_v = [-5,11,7];
predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w,chassis_v); 
expected_values.wheel_v_rear_left = [-31 11 11];
expected_values.wheel_v_rear_right = [21 11 3];
assert(isequal(predicted_values.wheel_v_rear_left,expected_values.wheel_v_rear_left));
assert(isequal(predicted_values.wheel_v_rear_right,expected_values.wheel_v_rear_right));

%% Case: Input velocities are 3x3 
    % will give a warning because slide and bounce are large
    fprintf(1,'Testing complex case with 3x3 velocities \n')
    pos_rear_left = [[0,2,0];[0,2,0]]; % Note that the position has to be of the same size as the angular velocity
    pos_rear_right = [[0,-2,0];[0,-2,0]]; % Note that the position has to be of the same size as the angular velocity
    chassis_w = [[2,3,13];[21,11,7]];
    chassis_v = [[-5,11,7];[0,5,0]];
    predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w,chassis_v);
    expected_values.wheel_v_rear_left = [[-31 11 11];[-14 5 42]];
    expected_values.wheel_v_rear_right = [[21 11 3];[14 5 -42]];
    assert(isequal(predicted_values.wheel_v_rear_left,expected_values.wheel_v_rear_left));
    assert(isequal(predicted_values.wheel_v_rear_right,expected_values.wheel_v_rear_right));

%% Error throws
if 1 == 0 
    pos_rear_left = [0,10,0]; % Position from center of wheel in Vehicle-body coordinates
    pos_rear_right = [0,-10,0]; % Position from center of wheel in Vehicle-body coordinates
    chassis_w = [0,0,90];
    %% Case: Incorrect number of inputs
    predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w); %#ok<NASGU>
    %% Case: Inputs not arrays
    chassis_v = 10;
    predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w, chassis_v); %#ok<NASGU>
    %% Case: Non-zero wheel position input for x and z components plus large bounce and slide 
    pos_rear_left = [7,10,9]; % Position from center of wheel in Vehicle-body coordinates
    pos_rear_right = [8,-10,3]; % Position from center of wheel in Vehicle-body coordinates
    chassis_w = [500,0,900];
    chassis_v = [0,200,0];
    predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w, chassis_v);
end
