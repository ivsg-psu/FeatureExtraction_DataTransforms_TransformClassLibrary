function [roll_array,pitch_array, yaw_array] = fcn_Transform_CalculateAnglesofRotation(GPS_SparkFun_LeftRear_ENU,GPS_SparkFun_RightRear_ENU,GPS_SparkFun_Front_ENU)


% Input
% GPS_SparkFun_LeftRear_ENU:  Nx3
% GPS_SparkFun_RightRear_ENU: Nx3
% GPS_SparkFun_Front_ENU:     Nx3

% The ISO convention is used for vehicle-affixed coordinate system
% Assume the Rear Left and Rear Right GPS Antennas are perfect aligned
% along the vehicle y axis

V_right_to_left = GPS_SparkFun_LeftRear_ENU - GPS_SparkFun_RightRear_ENU; % Nx3
V_right_to_left_mag = vecnorm(V_right_to_left,2,2); % Nx3
V_y_unit = V_right_to_left./V_right_to_left_mag; % Nx3

% The front GPS antenna is not aligned with any rear antenna along the
% vehicle x axis
V = GPS_SparkFun_Front_ENU - GPS_SparkFun_RightRear_ENU; % Nx3
V_projection = fcn_Transform_VectorProjection(V, V_y_unit); % The projection is along V_y_unit, Nx3
V_shift_y = V - V_projection; % Nx3
V_shift_y_mag = vecnorm(V_shift_y,2,2); % Nx3

% The front antenna is 4.2 cm lower than the rear right and rear left
% antenna
z_offset = -0.042; 
angle_diff = asin(z_offset./V_shift_y_mag);% Nx3
theta = -angle_diff;
% To make it aligned, rotate the vector about V_y_unit
V_aligned = V_shift_y.*cos(theta) + cross(V_y_unit,V_shift_y,2).*sin(theta)+...
             V_y_unit.*dot(V_y_unit,V_shift_y,2).*(1-cos(theta));
% V_aligned = V_y_unit.*dot(V_y_unit,V_shift_y,2) +... 
%             cross(cos(reverse_angle).*cross(V_y_unit,V_shift_y,2),V_shift_y,2)+...
%             sin(reverse_angle).*cross(V_y_unit,V_shift_y,2);
V_aligned_mag = vecnorm(V_aligned,2,2);
V_x_unit = V_aligned./V_aligned_mag;
V_z_unit = cross(V_x_unit,V_y_unit,2);
angle_90 = rad2deg(asin(vecnorm(cross(V_x_unit,V_y_unit,2),2,2)));

N_points = length(V_y_unit);
for n = 1:N_points
    R = [V_x_unit(n,:).' V_y_unit(n,:).' V_z_unit(n,:).'];
    roll = atan2(R(3,2),R(3,3));
    pitch = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
        
    yaw = atan2(R(2,1), R(1,1));
    roll_array(n,1) = roll;
    pitch_array(n,1) = pitch;
    yaw_array(n,1) = yaw;

end

