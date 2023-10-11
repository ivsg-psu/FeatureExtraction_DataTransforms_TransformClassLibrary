function vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU_PITCH(GPSLeft_ENU, GPSRight_ENU, SensorMount_offset_relative_to_VehicleOrigin, varargin)
% fcn_Transform_findVehiclePoseinENU
%
% This function takes two GPS Antenna centers, GPSLeft_ENU and 
% GPSRight_ENU, in ENU coordinates as a (1 x 3) vector representing 
% [x, y, z] in meters, PITCH_vehicle_ENU in degrees and the sensor 
% mount's offset relative to the vehicle's origin as
% 
% [SensorMount_offset_x_relative_to_VehicleOrigin, 
%  SensorMount_offset_y_relative_to_VehicleOrigin, 
%  SensorMount_offset_z_relative_to_VehicleOrigin] 
% 
% as the inputs and outputs the vehicle pose in ENU coordinates as 
% [X_vehicle_ENU, Y_vehicle_ENU, Z_vehicle_ENU, ROLL_vehicle_ENU, 
%  PITCH_vehicle_ENU, YAW_vehicle_ENU]
%
% ASSUMPTIONS: 
%
% 1 - The ISO convention is used to output the orientation of the vehicle
%
% 2 - SensorMount_offset_y_relative_to_VehicleOrigin = 0. 
%
% 3 - The distance between the GPS Antennas mid-point and the Sensor Mount 
%     is assumed to be zero 
%
% 4 - The distance between GPSRight_ENU to GPSLeft_ENU is 2 meters. 
%
% METHOD:
%
% Step 1: Find the YAW of the vehicle relative to ENU coordinates
%        (YAW_vehicle_ENU)
%
% 1) - Find the unit vector from GPSLeft_ENU --> GPSRight_ENU
%
% 2) - Find the orthogonal by rotating the unit vector by -90 degrees via 
%      a 3x3 matrix multiplication to determine vehicle orientation in Z 
%      axis of ENU coordinates 
%
% 3) - The YAW is calculated by finding the arc tangent of the orthogonal 
%      of the unit vector. The 4-quadrant output is returned as an angle 
%      in radians using atan2.
%
% Step 2: Find the ROLL of the vehicle relative to ENU coordinates
%        (ROLL_vehicle_ENU)
%
% 1) - The vehicle is rotated based on the YAW_vehicle_ENU found in the
%      previous step. The vehicle is rotated in a way that the YAW is zero.
%
% 2) - The vehicle is rotated based on the PITCH_vehicle_ENU given in the
%      input. The vehicle is rotated in a way that the PITCH is zero.
%
% 3) - Find the dot product of horizontal unit vector when YAW, PITCH, and 
%      ROLL are zero and the unit vector when YAW and PITCH are zero from 
%      left to right, and the corresponding norms are calculated to find 
%      the magnitude of the ROLL of the vehicle. acos function is used to
%      find the angle between the two unit vectors.
%
% 4) - The direction of the ROLL is obtained by calculating the cross
%      product of the two unit vectors mentioned above (horizontal unit 
%      vector when YAW, PITCH, and ROLL are zero and the unit vector when 
%      YAW and PITCH are zero from left to right). The sign of the
%      X-coordinate of the cross product determines the direction of the
%      ROLL.
%      
% Step 3: Find the Vehicle Origin, in ENU coordinates (vehicleOrigin_ENU)
%         based on YAW.
%
% 1) - Find the mid-point of the GPSLeft_ENU and GPSRight_ENU to find the
%      position (origin) of the sensor mount. 
%
% 2) - Translate the mid-point of the GPSLeft_ENU and GPSRight_ENU to 
%      vehicle's origin by assuming the "YAW" of the sensor mount relative 
%      to the vehicle as "zero", and assuming that the center of the 
%      vehicle is "ahead" in the vehicle coordinates of the sensor mount 
%      by a known distance. The projection from sensor mount to vehicle's 
%      origin is [+X_SensorMount_center, 0, -Z_SensorMount_center] 
%      assuming that the sensor mount is [-X_SensorMount_center, 0, 
%      +Z_SensorMount_center] relative to the vehicle's origin.
%
% Step 4: Find the Vehicle Pose in ENU coordinates
%   
%  * vehiclePose_ENU = [VehicleOrigin_ENU, ROLL_vehicle_ENU, PITCH_vehicle_ENU, 
%                       YAW_vehicle_ENU]. 
%
% FORMAT:
%
%      vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin)
%
% INPUTS:
%      
%      GPSLeft_ENU: the center of the left GPS Antenna [xLeft, yLeft, 
%      zLeft] in meters
%
%      GPSRight_ENU: the center of the right GPS Antenna [xRight, yRight, 
%      zRight] in meters
%
%      PITCH_vehicle_ENU: the PITCH of the vehicle in ENU coordinates in
%      degrees
%
%      SensorMount_offset_relative_to_VehicleOrigin: the position of the
%      sensor mount relative to the vehicle origin [-X_SensorMount_center,
%      0, +Z_SensorMount_center]
%                      
%
% (OPTIONAL INPUTS)   
%           
%     fig_num: The figure is plotted if fig_num is entered as the input. 
% 
% OUTPUTS:
%      
%      vehiclePose_ENU: the position of the vehicle's origin and
%      orientation of the vehicle in ENU coordinates as [X_vehicle_ENU, 
%      Y_vehicle_ENU, Z_vehicle_ENU, ROLL_vehicle_ENU, PITCH_vehicle_ENU, 
%      YAW_vehicle_ENU]
% 
% 
% DEPENDENCIES:
% 
%     None
% 
% EXAMPLES:
% 
%     See the script: script_test_fcn_findVehiclePoseinENU
%     for a full test suite.

% Revision history:
%     
% 2023_07_09: Aneesh Batchu
% -- wrote the code originally
% 2023_08_07: Aneesh Batchu
% Vectorized the code

% TO DO
% Should be able to find the Roll and Pitch of the vehicle in ENU
% coordinates - This is the later task. 

flag_do_debug = 0;  % Flag to show the results for debugging
flag_do_plots = 0;  % % Flag to plot the final results
flag_check_inputs = 1; % Flag to perform input checking
if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
end
%% check input arguments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if flag_check_inputs
    % Are there the right number of inputs?
    narginchk(4,5);

end

% Does user want to show the plots?
if 5 == nargin
    temp = varargin{end};
    if ~isempty(temp)
        fig_num = temp;
        figure(fig_num);
        flag_do_plots = 1; 
    end
else
    if flag_do_debug
        flag_do_plots = 1;
    end
end

%% Main code starts here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Step 1 - Finding the YAW of the vehicle relative to ENU coordinates

% Find the unit vector from GPSLeft_ENU --> GPSRight_ENU

unitVector_from_GPSLeft_to_GPSRight = (GPSRight_ENU - GPSLeft_ENU)./(sum((GPSLeft_ENU - GPSRight_ENU).^2,2).^0.5);

% Find the orthogonal (Rotate the unit vector by 90 degrees)

% Convert unit vector to homogeneous coordinates
onesColumn = ones(size(unitVector_from_GPSLeft_to_GPSRight, 1),1);
unitVector_from_GPSLeft_to_GPSRight = [unitVector_from_GPSLeft_to_GPSRight, onesColumn]';


% transform matrix to rotate the unit vector by -90 degrees
Mtr_rotate_negative90 = [0 -1 0 0; 1 0 0 0; 0 0 1 0; 0 0 0 1];

rotated_unitVector_from_1to2 = Mtr_rotate_negative90*(unitVector_from_GPSLeft_to_GPSRight);

% Find yaw - by finding arc tangent of the orthogonal of the unit vector

yaw_in_rad_ENU = atan2(rotated_unitVector_from_1to2(2,:), rotated_unitVector_from_1to2(1,:));
yaw_in_deg_ENU = rad2deg(yaw_in_rad_ENU);

% fprintf('The YAW of the vehicle in ENU coordinates is %.4f \n',yaw_in_deg_ENU);

%% Step 2 - Finding the ROLL of the vehicle relative to ENU coordinates

% Rotate the vehicle based on the YAW_vehicle_ENU found in the previous step. 
% The vehicle is rotated in a way that the YAW is zero.
yaw_to_zero_ENU = -yaw_in_rad_ENU;

% No. of elements in yaw_to_zero_ENU
Nelems_yaw_to_zero_ENU = size(yaw_to_zero_ENU, 2); 
ISO_roll_in_deg_ENU = zeros(Nelems_yaw_to_zero_ENU,1);
pitch_in_deg_ENU = zeros(Nelems_yaw_to_zero_ENU,1);
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
    horizontal_unitVector_left_to_right_yaw_zero = [0; -1; 0; 1];

    % Dot product of the pitch_to_zero unit vector and horizontal unit vector
   dot_product = dot(yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight(1:3),horizontal_unitVector_left_to_right_yaw_zero(1:3));

   % Magnitudes of the pitch_to_zero unit vector and horizontal unit vector
   norm_true_vector_from_GPSLeft_to_GPSRight = norm(yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight(1:3));
   norm_horizontal_vector_from_GPSLeft_to_GPSRight = norm(horizontal_unitVector_left_to_right_yaw_zero(1:3));

   % Finding the ROLL (theta value) using inverse cosine
   mag_roll_in_rad_ENU = acos(dot_product/(norm_true_vector_from_GPSLeft_to_GPSRight*norm_horizontal_vector_from_GPSLeft_to_GPSRight));

   % Find the cross product to determine the direction of the ROLL
   cross_product = cross(horizontal_unitVector_left_to_right_yaw_zero(1:3),yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight(1:3));

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
   

   if sign(rotated_unitVector_from_1to2(1,1)) == 1
       % transform matrix to rotate the unit vector by -90 degrees
       Mtr_rotate_90_roll_to_zero = [0 -1 0 0; 1 0 0 0; 0 0 1 0; 0 0 0 1];
   else
       Mtr_rotate_90_roll_to_zero = [0 1 0 0; -1 0 0 0; 0 0 1 0; 0 0 0 1];
   end

   rotated_roll_yaw_to_zero_unitVector = Mtr_rotate_90_roll_to_zero*roll_to_zero_unitVector_from_GPSLeft_to_GPSRight;

   % The homogeneous horizontal unitvector from GPSCenter_ENU --> GPSFront_ENU
   % when orientation is zero
   % This not correct vectoe. This unit vector 
   horizontal_unitVector_from_GPSCenter_to_Front_Orientation_Zero = [1*sign(rotated_unitVector_from_1to2(1,1)); 0; 0; 1]; % This needs to be changed

   % Dot product of the pitch_to_zero unit vector and horizontal unit vector
   dot_product = dot(rotated_roll_yaw_to_zero_unitVector(1:3),horizontal_unitVector_from_GPSCenter_to_Front_Orientation_Zero(1:3));

   % Magnitudes of the pitch_to_zero unit vector and horizontal unit vector
   norm_unitVector_from_GPSCenter_to_GPSFront_roll_yaw_zero = norm(rotated_roll_yaw_to_zero_unitVector(1:3));
   norm_horizontal_unitVector_from_GPSCenter_to_Front_OrientZero = norm(horizontal_unitVector_from_GPSCenter_to_Front_Orientation_Zero(1:3));

   % Finding the PITCH (theta value) using inverse cosine
   mag_pitch_in_rad_ENU = acos(dot_product/(norm_unitVector_from_GPSCenter_to_GPSFront_roll_yaw_zero*norm_horizontal_unitVector_from_GPSCenter_to_Front_OrientZero));

   % Find the cross product to determine the direction of the PITCH
   cross_product = cross(rotated_roll_yaw_to_zero_unitVector(1:3),horizontal_unitVector_from_GPSCenter_to_Front_Orientation_Zero(1:3));

   % The direction of the pitch depends on the Y - coordinate of the cross
   % product
   direction_of_PITCH = sign(cross_product(2));

   % Pitch of the vehicle in ENU coordinates in radians
   pitch_in_rad_ENU = direction_of_PITCH*mag_pitch_in_rad_ENU;

   % Roll of the vehicle in ENU coordinates in degrees
   pitch_in_deg_ENU(i,1) = rad2deg(pitch_in_rad_ENU);





   %% Step 3 - Find the Vehicle Origin, in ENU coordinates based on YAW

   % Find the center of the sensor mount
   GPS_center = (GPSLeft_ENU(i,:) + GPSRight_ENU(i,:))/2;

   % Find the vehicle offset relative to sensor mount
   VehicleOrigin_offset_relative_to_SensorMount = -SensorMount_offset_relative_to_VehicleOrigin;


   Translate_midPoint_of_SensorMount_to_VehicleOrigin = VehicleOrigin_offset_relative_to_SensorMount(1)*rotated_unitVector_from_1to2(1:3,i)';

   vehicleOrigin_ENU(i,1) = GPS_center(1) + Translate_midPoint_of_SensorMount_to_VehicleOrigin(1);
   vehicleOrigin_ENU(i,2) = GPS_center(2) + Translate_midPoint_of_SensorMount_to_VehicleOrigin(2);
   vehicleOrigin_ENU(i,3) = GPS_center(3) + Translate_midPoint_of_SensorMount_to_VehicleOrigin(3) + VehicleOrigin_offset_relative_to_SensorMount(3);

end

%% Step 4 - Find the Vehicle POSE in ENU coordinates

vehiclePose_ENU = [vehicleOrigin_ENU, ISO_roll_in_deg_ENU, pitch_in_deg_ENU, yaw_in_deg_ENU'];

fprintf(1,'\nThe POSE of the vehicle in ENU coordinates is :\n');
disp(vehiclePose_ENU);

%% Plot the results (for debugging)?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _
%  |  __ \     | |
%  | |  | | ___| |__  _   _  __ _
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag_do_plots
    
    oriGin = [0, 0, 0];
    
    % Plot the Left GPS center
    quiver3(oriGin(1,1), oriGin(1,2), oriGin(1,3), GPSLeft_ENU(1,1), GPSLeft_ENU(1,2), GPSLeft_ENU(1,3), 'Color', 'b', 'LineWidth', 1);
    plot3(GPSLeft_ENU(1,1), GPSLeft_ENU(1,2), GPSLeft_ENU(1,3), 'r.', 'MarkerSize', 50);
    
    % Plot the Right GPS center 
    quiver3(oriGin(1,1), oriGin(1,2), oriGin(1,3), GPSRight_ENU(1,1), GPSRight_ENU(1,2), GPSRight_ENU(1,3), 'Color', 'g', 'LineWidth', 1);
    plot3(GPSRight_ENU(1,1), GPSRight_ENU(1,2), GPSRight_ENU(1,3), 'r.', 'MarkerSize', 50);

    % Connecting Left GPS center and Left GPS center with a line
    line_data = [GPSLeft_ENU; GPSRight_ENU];
    plot3(line_data(:,1), line_data(:,2), line_data(:,3), 'r-', 'LineWidth', 3);

    % Plot the direction of the unit vector 
    quiver3(oriGin(1,1), oriGin(1,2), oriGin(1,3), unitVector_from_GPSLeft_to_GPSRight(1,1), unitVector_from_GPSLeft_to_GPSRight(2,1), unitVector_from_GPSLeft_to_GPSRight(3,1), 'Color', 'k', 'LineWidth', 1);
    
    % Plot the direction of the orthoginal of the unit vector
    quiver3(oriGin(1,1), oriGin(1,2), oriGin(1,3), rotated_unitVector_from_1to2(1,1), rotated_unitVector_from_1to2(2,1), rotated_unitVector_from_1to2(3,1), 'Color', 'r', 'LineWidth', 1);
    
    % Plot the Sensor Mount origin
    plot3(GPS_center(1,1), GPS_center(1,2), GPS_center(1,3), 'b.', 'MarkerSize', 50);
    
    % Plot the unitvector from GPSLeft_to_GPSRight when YAW is zero
    quiver3(oriGin(1,1), oriGin(1,2), oriGin(1,3), yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight(1), yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight(2), yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight(3), 'Color', 'green', 'LineWidth', 2);

    % Plot the unitvector from GPSLeft_to_GPSRight when YAW and PITCH are zero
    % quiver3(oriGin(1,1), oriGin(1,2), oriGin(1,3), yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight(1), yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight(2), yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight(3), 'Color', 'magenta', 'LineWidth', 2);

    % Direction of the unit horizontal vector when YAW, PITCH and ROLL are zero
    quiver3(oriGin(1,1), oriGin(1,2), oriGin(1,3), horizontal_unitVector_left_to_right_yaw_zero(1), horizontal_unitVector_left_to_right_yaw_zero(2), horizontal_unitVector_left_to_right_yaw_zero(3), 'Color', 'red', 'LineWidth', 2);

    % Plot the unitvector from GPSLeft_to_GPSRight when YAW is zero
    quiver3(oriGin(1,1), oriGin(1,2), oriGin(1,3), roll_to_zero_unitVector_from_GPSLeft_to_GPSRight(1), roll_to_zero_unitVector_from_GPSLeft_to_GPSRight(2), roll_to_zero_unitVector_from_GPSLeft_to_GPSRight(3), 'Color', 'c', 'LineWidth', 2);
    
    % Find the GPSRight_ENU such that YAW of the vehicle is zero
    GPSRight_ENU_orient_to_zero = [GPSLeft_ENU(1), GPSLeft_ENU(2) + 2*roll_to_zero_unitVector_from_GPSLeft_to_GPSRight(2), GPSLeft_ENU(3)];
    plot3(GPSRight_ENU_orient_to_zero(1), GPSRight_ENU_orient_to_zero(2), GPSRight_ENU_orient_to_zero(3), 'c.', 'MarkerSize', 50);
        
    
    % Plot the Vehicle origin 
    plot3(vehicleOrigin_ENU(1,1), vehicleOrigin_ENU(1,2), vehicleOrigin_ENU(1,3), 'g.', 'MarkerSize', 50);
end

if flag_do_debug
    fprintf(fileID,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end