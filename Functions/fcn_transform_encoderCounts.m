% take #counts, encoder counts/rev, max count, min count, initial encoder
% value as input --> later ---> predict encoder position 
predicted_values.wheel_v_rear_left = wheel_v_rear_left;
predicted_values.wheel_v_rear_right = wheel_v_rear_right;
predicted_values.encoder_w_rear_left = encoder_w_rear_left;
predicted_values.encoder_w_rear_right = encoder_w_rear_right;
predicted_values.tire_slide_rear_left = tire_slide_rear_left;
predicted_values.tire_slide_rear_right = tire_slide_rear_right; 
predicted_values.tire_slide_rear_right = tire_slide_rear_right;
predicted_values.vehicle_bounce_rear_left = vehicle_bounce_rear_left; 
predicted_values.vehicle_bounce_rear_right = vehicle_bounce_rear_right; 
predicted_values.wheel_acceleration_rear_left = wheel_acceleration_rear_left;
predicted_values.wheel_acceleration_rear_right = wheel_acceleration_rear_right; 
%Find acceleration  
wheel_acceleration_rear_left = vehicle_acceleration + cross(chassis_alpha,pos_rear_left); 
wheel_acceleration_rear_right = vehicle_acceleration + cross(chassis_alpha,pos_rear_right);

encoder_w_rear_left = wheel_v_rear_left(1) ./ tire_radius;
encoder_w_rear_right = wheel_v_rear_right(1) ./ tire_radius;

% INPUTS:
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
%       chassis_alpha: angular acceleration of vehicle chassis in rad/s^2;
%       vector; expected input is an array, [a_x, a_y, a_z], where a_x,
%       a_y, and a_z are components of the chassis' angular acceleration.
%
%       chassis_v: linear velocity of vehicle in m/s; vector; expected input is an
%       array, [v_x, v_y, v_z], where v_x, v_y, v_z are components of the
%       chassis' linear velocity.
%
%       vehicle_acceleration: linear acceleration of vehicle in m/s^2; vector;
%       expected input is an array,[a_x, a_y, a_z], where a_x, a_y, a_z are components of the
%       chassis' linear acceleration.
%
%       tire_radius: radius of tire in meters. 