function predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w,chassis_v)
% fcn_transform_predictWheelVelocity 
% This function predicts the velocity of the two rear wheels of the vehicle
%
% FORMAT:
%
%       predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w,chassis_v)
%
% INPUTS:
%
%
%       pos_rear_left: [x,y,z] position of rear left tire in meters. Expected value is
%                      [0, d, 0], where d is the distance from middle of rear vehicle axle to
%                      middle of tire
%
%       pos_rear_right: [x,y,z] position of rear right tire in meters. Expected value is
%                      [0, - d, 0], where d is the distance from middle of rear vehicle axle to
%                      middle of tire
% 
%       chassis_w: angular velocity of vehicle chassis in rad/s; vector; expected
%       input is an array, [w_x, w_y,w_z], where w_x, w_y, and w_z are the
%       components of the chassis' angular velocity.
%
%       chassis_v: linear velocity of vehicle in m/s; vector; expected input is an
%       array, [v_x, v_y, v_z], where v_x, v_y, v_z are components of the
%       chassis' linear velocity.
%             
% OUTPUTS:
%
%       wheel_v_rear_left: velocity of rear left wheel in m/s; vector, [V_x, V_y, V_z], where V_x, V_y, V_z are the
%       components of the linear velocity of the rear left wheel. 
%       wheel_v_rear_right: velocity of rear right wheel in m/s; vector, [V_x, V_y, V_z], where V_x, V_y, V_z are the
%       components of the linear velocity of the rear left wheel. 
%       
% DEPENDENCIES:
%
%       No dependencies 
%
% EXAMPLES:
%
%     See the script: script_test_fcn_transform_predictWheelVelocity
%     for a full test suite.

% Revision history:
%     
% 2023_06_22: Mariam Abdellatief
% -- Wrote the code originally 
% 2023_06_23: Mariam Abdellatief
% -- Added acceleration, tire slide, and vehicle bounce
% 2023_06_26: Mariam Abdellatief 
% -- Functionalized code
% 2023_06_27: Mariam Abdellatief 
% -- Added vehicle bounce and tire slide warnings 
% -- Added input arguments' checks (number of inputs, numeric or not, array or not) 
% -- Wrote test script
% 2023_6_28/29: Mariam Abdellatief
% -- Simplified function and added more test cases

%% Inputs
%Check number of inputs
if nargin < 4 || nargin > 4
        error ('Incorrect number of input arguments')
end 

%Check if inputs are arrays
arguments_list = {pos_rear_left,pos_rear_right,chassis_w,chassis_v};
for i = 1:numel(arguments_list)
    x = size(chassis_v,1);
    if ~isnumeric(arguments_list{i}) || ~isequal(size(arguments_list{i}), [x 3])
        error('One or more input arguments are not valid 1x3 arrays or is non-numeric.');      
    end
end 

%Check if the x and z components of wheel position is non-zero
if pos_rear_left(1,1)~=0 || pos_rear_left(1,3)~=0 || pos_rear_right(1,1)~=0 || pos_rear_right(1,3)~=0
    error("Wheel position components are expected to be zero in x and z.");
end

%% Main
%Find wheel's linear velocity 
wheel_v_rear_left = chassis_v + cross (chassis_w,pos_rear_left);
wheel_v_rear_right = chassis_v + cross (chassis_w,pos_rear_right);

% Check vehicle bounce and tire slide 
if any(abs(wheel_v_rear_left(2)) > 0.2) || any(abs(wheel_v_rear_right(2)) > 0.2) %Cut-offs are not actual values 
    warning('Tire slide is greater than 0.2 \n')
end

if any(abs(wheel_v_rear_left(3)) > 0.2) || any(abs(wheel_v_rear_right(3)) > 0.2) %Cut-offs are not actual values 
    warning('Vehicle bounce is greater than 0.2 \n')
end 

%Organize output 
predicted_values.wheel_v_rear_left = wheel_v_rear_left;
predicted_values.wheel_v_rear_right = wheel_v_rear_right;

end  

