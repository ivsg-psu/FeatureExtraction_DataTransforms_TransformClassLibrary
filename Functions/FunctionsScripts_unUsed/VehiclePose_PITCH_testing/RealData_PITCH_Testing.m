clc
close all

% The center of Left GPS Antenna
% GPSLeft_ENU = [x, y, z] in meters
GPSLeft_ENU = LeftGPS_ENU(100,:);

% The center of Right GPS Antenna
% GPSRight_ENU = [x, y, z] in meters
GPSRight_ENU = RightGPS_ENU(100,:);

% The center of Front GPS Antenna
% GPSRight_ENU = [x, y, z] in meters
GPSFront_ENU = FrontGPS_ENU(100,:);


figure(343434)
plot3(GPSLeft_ENU(:,1),GPSLeft_ENU(:,2),GPSLeft_ENU(:,3),'b-','Linewidth',3);
hold on
plot3(GPSRight_ENU(:,1),GPSRight_ENU(:,2),GPSRight_ENU(:,3),'r-','Linewidth',3);
plot3(GPSFront_ENU(:,1),GPSFront_ENU(:,2),GPSFront_ENU(:,3),'-','Linewidth',3);
grid on

% The sensor mount offset relative to vehicle origin = 
% [-X_SensorMount_center, 0, +Z_SensorMount_center]
SensorMount_offset_relative_to_VehicleOrigin = [-0.6027 0 1.5261]; 

GPS_center = (GPSLeft_ENU + GPSRight_ENU)/2;

% % The center of Front GPS Antenna
% % GPSFront_ENU = [x, y, z] in meters
% GPSFront_ENU =[GPS_center(1,1)+2, GPS_center(1,2), GPS_center(1,3)];



figure(898)

clf;
grid on
hold on
axis equal
xlabel('East', 'FontSize',15, 'FontWeight','bold')
ylabel('North', 'FontSize',15, 'FontWeight','bold')
zlabel('Up', 'FontSize',15, 'FontWeight','bold')
view(3)



% text(x(2) + 0.1, y(2), 'Point B', 'FontSize', 12);
% text(x(3) + 0.1, y(3), 'Point C', 'FontSize', 12);

oriGin = [0, 0, 0];

% Plot the Left GPS center
quiver3(oriGin(1,1), oriGin(1,2), oriGin(1,3), GPSLeft_ENU(1,1), GPSLeft_ENU(1,2), GPSLeft_ENU(1,3), 'Color', 'b', 'LineWidth', 1);
plot3(GPSLeft_ENU(1,1), GPSLeft_ENU(1,2), GPSLeft_ENU(1,3), 'r.', 'MarkerSize', 50);
text(GPSLeft_ENU(1,1) + 0.1, GPSLeft_ENU(1,2) + 0.1, GPSLeft_ENU(1,3) + 0.1, 'Left GPS', 'FontSize', 12);

% Plot the Right GPS center 
quiver3(oriGin(1,1), oriGin(1,2), oriGin(1,3), GPSRight_ENU(1,1), GPSRight_ENU(1,2), GPSRight_ENU(1,3), 'Color', 'g', 'LineWidth', 1);
plot3(GPSRight_ENU(1,1), GPSRight_ENU(1,2), GPSRight_ENU(1,3), 'r.', 'MarkerSize', 50);
text(GPSRight_ENU(1,1) + 0.1, GPSRight_ENU(1,2) + 0.1, GPSRight_ENU(1,3) + 0.1, 'Right GPS', 'FontSize', 12);


% Connecting Left GPS center and Left GPS center with a line
rearline_data = [GPSLeft_ENU; GPSRight_ENU];
plot3(rearline_data(:,1), rearline_data(:,2), rearline_data(:,3), 'r-', 'LineWidth', 3)

% Plot the GPS Center
plot3(GPS_center(1,1), GPS_center(1,2), GPS_center(1,3), 'b.', 'MarkerSize', 50);

% Plot the GPS Center - Front GPS
plot3(GPS_center(1,1), GPS_center(1,2)+0.6, GPS_center(1,3)-0.1, 'b.', 'MarkerSize', 50);

% CODE

unitVector_from_GPSLeft_to_GPSRight = (GPSRight_ENU - GPSLeft_ENU)./(sum((GPSLeft_ENU - GPSRight_ENU).^2,2).^0.5);

% Convert unit vector to homogeneous coordinates
onesColumn = ones(size(unitVector_from_GPSLeft_to_GPSRight, 1),1);
unitVector_from_GPSLeft_to_GPSRight = [unitVector_from_GPSLeft_to_GPSRight, onesColumn]';

% transform matrix to rotate the unit vector by -90 degrees
Mtr_rotate_negative90 = [0 -1 0 0; 1 0 0 0; 0 0 1 0; 0 0 0 1];

rotated_unitVector_from_1to2 = Mtr_rotate_negative90*(unitVector_from_GPSLeft_to_GPSRight);

% Find yaw - by finding arc tangent of the orthogonal of the unit vector

yaw_in_rad_ENU = atan2(rotated_unitVector_from_1to2(2,:), rotated_unitVector_from_1to2(1,:));
yaw_in_deg_ENU = rad2deg(yaw_in_rad_ENU);

% The center of Front GPS Antenna
% GPSFront_ENU = [x, y, z] in meters

Translate_GPSCenter_to_FrontGPSCenter = 2*rotated_unitVector_from_1to2(1:3,1)';

GPSFront_ENU =[GPS_center(1,1), GPS_center(1,2)+0.46, GPS_center(1,3) - 0.1] + Translate_GPSCenter_to_FrontGPSCenter;

% Plot the Front GPS center 
quiver3(oriGin(1,1), oriGin(1,2), oriGin(1,3), GPSFront_ENU(1,1), GPSFront_ENU(1,2), GPSFront_ENU(1,3), 'Color', 'k', 'LineWidth', 1);
plot3(GPSFront_ENU(1,1), GPSFront_ENU(1,2), GPSFront_ENU(1,3), 'r.', 'MarkerSize', 50);
text(GPSFront_ENU(1,1) + 0.1, GPSFront_ENU(1,2) + 0.1, GPSFront_ENU(1,3) + 0.1, 'Front GPS', 'FontSize', 12);

% Connecting GPS center and Front GPS center with a line
Front_GPSCenter_line_data = [GPS_center; GPSFront_ENU];
plot3(Front_GPSCenter_line_data(:,1), Front_GPSCenter_line_data(:,2), Front_GPSCenter_line_data(:,3), 'r-', 'LineWidth', 3)



% Rotate the vehicle based on the YAW_vehicle_ENU found in the previous step. 
% The vehicle is rotated in a way that the YAW is zero.
yaw_to_zero_ENU = -yaw_in_rad_ENU;

% No. of elements in yaw_to_zero_ENU
Nelems_yaw_to_zero_ENU = size(yaw_to_zero_ENU, 2); 
pitch_in_deg_ENU = zeros(Nelems_yaw_to_zero_ENU,1);
ISO_roll_in_deg_ENU = zeros(Nelems_yaw_to_zero_ENU,1);
vehicleOrigin_ENU = zeros(Nelems_yaw_to_zero_ENU,1);


for i = 1:Nelems_yaw_to_zero_ENU

% Transform matrix to zrotate "YAW" in negative Yaw_in_rad_ENU
    Mtr_yaw_to_zero = [cos(yaw_to_zero_ENU(1,i)), -sin(yaw_to_zero_ENU(1,i)), 0, 0;
                       sin(yaw_to_zero_ENU(1,i)), cos(yaw_to_zero_ENU(1,i)), 0, 0;
                       0, 0, 1, 0;
                       0, 0, 0, 1];

    % Rotate the unit vector in the direction, where YAW of the vehicle is zero
    % in ENU
    yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight = Mtr_yaw_to_zero*unitVector_from_GPSLeft_to_GPSRight(:,i);

    % The homogeneous horizontal unitvector from GPSLeft_ENU --> GPSRight_ENU 
    % when YAW is zero
    horizontal_unitVector_left_to_right_yaw_pitch_zero = [0; -1; 0; 1];

    % Dot product of the pitch_to_zero unit vector and horizontal unit vector
   dot_product = dot(yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight(1:3),horizontal_unitVector_left_to_right_yaw_pitch_zero(1:3));

   % Magnitudes of the pitch_to_zero unit vector and horizontal unit vector
   norm_true_vector_from_GPSLeft_to_GPSRight = norm(yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight(1:3));
   norm_horizontal_vector_from_GPSLeft_to_GPSRight = norm(horizontal_unitVector_left_to_right_yaw_pitch_zero(1:3));

   % Finding the ROLL (theta value) using inverse cosine
   mag_roll_in_rad_ENU = acos(dot_product/(norm_true_vector_from_GPSLeft_to_GPSRight*norm_horizontal_vector_from_GPSLeft_to_GPSRight));

   % Find the cross product to determine the direction of the ROLL
   cross_product = cross(horizontal_unitVector_left_to_right_yaw_pitch_zero(1:3),yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight(1:3));

   % The direction of the roll depends on the X - coordinate of the cross
   % product
   direction_of_ROLL = sign(cross_product(1));

   % Roll of the vehicle in ENU coordinates in radians
   roll_in_rad_ENU = direction_of_ROLL*mag_roll_in_rad_ENU;

   % Roll of the vehicle in ENU coordinates in degrees
   roll_in_deg_ENU = rad2deg(roll_in_rad_ENU);

   % -1 is multiplied as the orientation of the vehicle follows the ISO
   % convention
   ISO_roll_in_deg_ENU(i,1) = -1*roll_in_deg_ENU;

   roll_to_zero_ENU = -roll_in_rad_ENU;

   % Roll transform matrix
   Mtr_roll_to_zero = [1, 0, 0, 0; 0 cos(roll_to_zero_ENU), -sin(roll_to_zero_ENU), 0; 0, sin(roll_to_zero_ENU), cos(roll_to_zero_ENU), 0; 0, 0, 0, 1];

   % Rotate the unit vector in the direction, where YAW and ROLL of the
   % vehicle are zero in ENU
   roll_to_zero_unitVector_from_GPSLeft_to_GPSRight = Mtr_roll_to_zero*yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight;

   % Plot the unitVector_from_GPSCenter_to_GPSFront
   quiver3(oriGin(1), oriGin(2), oriGin(3), roll_to_zero_unitVector_from_GPSLeft_to_GPSRight(1,1), roll_to_zero_unitVector_from_GPSLeft_to_GPSRight(2,1), roll_to_zero_unitVector_from_GPSLeft_to_GPSRight(3,1), 'Color', 'm', 'LineWidth', 1);

   if sign(rotated_unitVector_from_1to2(1,1)) == 1
       % transform matrix to rotate the unit vector by -90 degrees
       Mtr_rotate_90_roll_to_zero = [0 -1 0 0; 1 0 0 0; 0 0 1 0; 0 0 0 1];
   else
       Mtr_rotate_90_roll_to_zero = [0 1 0 0; -1 0 0 0; 0 0 1 0; 0 0 0 1];
   end

   rotated_roll_to_zero_unitVector = Mtr_rotate_90_roll_to_zero*roll_to_zero_unitVector_from_GPSLeft_to_GPSRight;

   % Plot the unitVector_from_GPSCenter_to_GPSFront
   quiver3(oriGin(1), oriGin(2), oriGin(3), rotated_roll_to_zero_unitVector(1,1), rotated_roll_to_zero_unitVector(2,1), rotated_roll_to_zero_unitVector(3,1), 'Color', 'c', 'LineWidth', 1);

   % Change this
   % Find the unitvector from a point between the left and right GPS and
   % front GPS
   % The point should be in the line of the Front GPS. 
   unitVector_from_GPSCenter_to_GPSFront = (GPSFront_ENU - GPS_center)./(sum((GPSFront_ENU - GPS_center).^2,2).^0.5);
   % Convert unit vector to homogeneous coordinates
   onesColumn = ones(size(unitVector_from_GPSCenter_to_GPSFront, 1),1);
   unitVector_from_GPSCenter_to_GPSFront = [unitVector_from_GPSCenter_to_GPSFront, onesColumn]';
   % Plot the unitVector_from_GPSCenter_to_GPSFront
   quiver3(oriGin(1), oriGin(2), oriGin(3), unitVector_from_GPSCenter_to_GPSFront(1,1), unitVector_from_GPSCenter_to_GPSFront(2,1), unitVector_from_GPSCenter_to_GPSFront(3,1), 'Color', 'k', 'LineWidth', 1);

   % Dot product of the pitch_to_zero unit vector and horizontal unit vector
   dot_product = dot(unitVector_from_GPSCenter_to_GPSFront(1:3),rotated_roll_to_zero_unitVector(1:3));

   % Magnitudes of the pitch_to_zero unit vector and horizontal unit vector
   norm_true_vector_from_GPSCenter_to_GPSFront = norm(unitVector_from_GPSCenter_to_GPSFront(1:3));
   norm_rotated_roll_to_zero_unitVector_from_GPSCenter_to_GPSFront = norm(rotated_roll_to_zero_unitVector(1:3));

   % Finding the PITCH (theta value) using inverse cosine
   mag_pitch_in_rad_ENU = acos(dot_product/(norm_true_vector_from_GPSCenter_to_GPSFront*norm_rotated_roll_to_zero_unitVector_from_GPSCenter_to_GPSFront));

   % Find the cross product to determine the direction of the PITCH
   cross_product = cross(rotated_roll_to_zero_unitVector(1:3),unitVector_from_GPSCenter_to_GPSFront(1:3));

   % The direction of the pitch depends on the Y - coordinate of the cross
   % product
   direction_of_PITCH = sign(cross_product(2));

   % Pitch of the vehicle in ENU coordinates in radians
   pitch_in_rad_ENU = direction_of_PITCH*mag_pitch_in_rad_ENU;

   % Roll of the vehicle in ENU coordinates in degrees
   pitch_in_deg_ENU(i,1) = rad2deg(pitch_in_rad_ENU);

   % Transform matrix to yrotate "PITCH" in negative PITCH_vehicle_ENU
   pitch_to_zero_ENU = -pitch_in_rad_ENU;
   Mtr_pitch_to_zero = [cos(pitch_to_zero_ENU), 0, sin(pitch_to_zero_ENU), 0;
                         0, 1, 0, 0;
                         sin(pitch_to_zero_ENU), 0, cos(pitch_to_zero_ENU), 0;
                         0, 0, 0, 1]; 
 


    % The homogeneous horizontal unitvector from GPSCenter_ENU --> GPSFront_ENU
    % when orientation is zero
    % horizontal_unitVector_center_to_front_orientation_zero = [1*sign(rotated_unitVector_from_1to2(1,1)); 0; 0; 1];

    % Plot the unitVector_from_GPSCenter_to_GPSFront 
    % quiver3(oriGin(1), oriGin(2), oriGin(3), horizontal_unitVector_center_to_front_orientation_zero(1), horizontal_unitVector_center_to_front_orientation_zero(2), horizontal_unitVector_center_to_front_orientation_zero(3), 'Color', 'b', 'LineWidth', 1);



   % Find the center of the sensor mount
   GPS_center = (GPSLeft_ENU(i,:) + GPSRight_ENU(i,:))/2;

   % Find the vehicle offset relative to sensor mount
   VehicleOrigin_offset_relative_to_SensorMount = -SensorMount_offset_relative_to_VehicleOrigin;


   Translate_midPoint_of_SensorMount_to_VehicleOrigin = VehicleOrigin_offset_relative_to_SensorMount(1)*rotated_unitVector_from_1to2(1:3,i)';

   vehicleOrigin_ENU(i,1) = GPS_center(1) + Translate_midPoint_of_SensorMount_to_VehicleOrigin(1);
   vehicleOrigin_ENU(i,2) = GPS_center(2) + Translate_midPoint_of_SensorMount_to_VehicleOrigin(2);
   vehicleOrigin_ENU(i,3) = GPS_center(3) + Translate_midPoint_of_SensorMount_to_VehicleOrigin(3) + VehicleOrigin_offset_relative_to_SensorMount(3);

end

vehiclePose_ENU = [vehicleOrigin_ENU, ISO_roll_in_deg_ENU, pitch_in_deg_ENU, yaw_in_deg_ENU'];

fprintf(1,'\nThe POSE of the vehicle in ENU coordinates is :\n');
disp(vehiclePose_ENU);