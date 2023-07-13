function vehiclePose_ENU = fcn_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, SensorMount_offset_relative_to_VehicleOrigin)
% fcn_findVehiclePoseinENU
%
% This function takes two GPS Antenna centers, GPSLeft_ENU and 
% GPSRight_ENU, in ENU coordinates as a (1 x 3) vector representing 
% [x, y, z] in meters and the sensor mount's offset relative to the 
% vehicle's origin as
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
% 1 - The vehicle is assumed to have zero roll and pitch in ENU coordinates. 
% 
% Therefore, the output of this code will be the position of the vehicle
% and YAW_vehicle_ENU.
% vehiclePose_ENU = [X_vehicle_ENU, Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, 
%                   YAW_vehicle_ENU(in degrees)]
%
% 2 - SensorMount_offset_y_relative_to_VehicleOrigin = 0. 
%
% 3 - Since the roll and pitch of the vehicle are assumed to be zero,
%     Z coordinates of the GPS Antenna centers are assumed to be " 2 " (2
%     meters from the ground). 
%
% 4 - The distance between the GPS Antennas mid-point and the Sensor Mount 
%     is assumed to be zero 
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
% Step 2: Find the Vehicle Origin, in ENU coordinates (vehicleOrigin_ENU)
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
% Step 3: Find the Vehicle Pose in ENU coordinates
%   
%  * vehiclePose_ENU = [VehicleOrigin_ENU, 0, 0, YAW_vehicle_ENU]. 
%
% FORMAT:
%
%      vehiclePose_ENU = fcn_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, SensorMount_offset_relative_to_VehicleOrigin)
%
% INPUTS:
%      
%      GPSLeft_ENU: the center of the left GPS Antenna [xLeft, yLeft, 
%      zLeft] in meters
%
%      GPSRight_ENU: the center of the right GPS Antenna [xRight, yRight, 
%      zRight] in meters
%
%      SensorMount_offset_relative_to_VehicleOrigin: the position of the
%      sensor mount relative to the vehicle origin [-X_SensorMount_center,
%      0, +Z_SensorMount_center]
%                      
%
%      (OPTIONAL INPUTS)     
% 
% OUTPUTS:
%      
%      vehiclePose_ENU: the position of the vehicle's origin and
%      orientation of the vehicle in ENU coordinates as [X_vehicle_ENU, 
%      Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]
% 
% 
% DEPENDENCIES:
% 
%      fcn_DebugTools_checkInputsToFunctions
% 
% EXAMPLES:
% 
%     See the script: script_test_fcn_findVehiclePoseinENU
%     for a full test suite.

% Revision history:
%     
% 2023_07_09: Aneesh Batchu
% -- wrote the code originally

% TO DO
% Should be able to find the Roll and Pitch of the vehicle in ENU
% coordinates - This is the later task. 

flag_do_debug = 0;  % Flag to show the results for debugging
flag_do_plots = 1;  % % Flag to plot the final results
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
    if nargin < 3 || nargin > 3
        error('Incorrect number of input arguments')
    end
        
    % NOTE: zone types are checked below

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

unitVector_from_GPSLeft_to_GPSRight = (GPSRight_ENU - GPSLeft_ENU)/(sum((GPSLeft_ENU - GPSRight_ENU).^2,2).^0.5);

% Find the orthogonal (Rotate the unit vector by -90 degrees)

% Convert unit vector to homogeneous coordinates
unitVector_from_GPSLeft_to_GPSRight = [unitVector_from_GPSLeft_to_GPSRight, 1]';

% transform matrix to rotate the unit vector by -90 degrees
Mtr_rotate_negative90 = [0 -1 0 0; 1 0 0 0; 0 0 1 0; 0 0 0 1];

rotated_unitVector_from_1to2 = Mtr_rotate_negative90*(unitVector_from_GPSLeft_to_GPSRight);

% % Verifying that the dot product is zero. 
% dot(unitVector_from_1to2(1:2), rotated_unitVector_from_1to2(1:2)');

% Find yaw - by finding arc tangent of the orthogonal of the unit vector

yaw_in_rad = atan2(rotated_unitVector_from_1to2(2), rotated_unitVector_from_1to2(1));
yaw_in_deg = rad2deg(yaw_in_rad);

fprintf('The YAW of the vehicle in ENU coordinates is %.4f \n',yaw_in_deg);

%% Step 3 - Find the Vehicle Origin, in ENU coordinates based on YAW

% Find the center of the sensor mount
SensorMount_center = (GPSLeft_ENU + GPSRight_ENU)/2;

% % Find the center of the sensor mount
% GPSAntennas_center = (GPSLeft_ENU + GPSRight_ENU)/2;
% 
% % The distance between the GPS Antennas mid-point and the Sensor Mount is
% % 0.1 meters
% dist_from_GPSAntennas_center_to_SensorMount_center = 0.1;
% 
% SensorMount_center = [GPSAntennas_center(1), GPSAntennas_center(2), GPSAntennas_center(3) - dist_from_GPSAntennas_center_to_SensorMount_center];

% Find the vehicle offset relative to sensor mount
VehicleOrigin_offset_relative_to_SensorMount = -SensorMount_offset_relative_to_VehicleOrigin;

% Translate the mid-point of the GPSLeft_ENU and GPSRight_ENU to vehicle
% origin by assuming the "YAW" of the sensor mount relative to the vehicle
% as "zero", and assuming that the center of the vehicle is "ahead" in the
% vehicle coordinates of the sensor mount by a known distance. The
% projection from sensor mount to vehicle's origin is [+X_SensorMount_center,
% 0, -Z_SensorMount_center] assuming that the sensor mount is 
% [-X_SensorMount_center, 0, +Z_SensorMount_center] relative to the 
% vehicle's origin.

% Translate the mid-point of the sensor mount in the direction of the
% orthogonal of the unit vector 

Translate_midPoint_of_SensorMount_to_VehicleOrigin = VehicleOrigin_offset_relative_to_SensorMount(1)*rotated_unitVector_from_1to2(1:2)';

vehicleOrigin_ENU = [SensorMount_center(1) + Translate_midPoint_of_SensorMount_to_VehicleOrigin(1), SensorMount_center(2) + Translate_midPoint_of_SensorMount_to_VehicleOrigin(2), SensorMount_center(3) + VehicleOrigin_offset_relative_to_SensorMount(3)]; 

%% Step 4 - Find the Vehicle POSE in ENU coordinates

vehiclePose_ENU = [vehicleOrigin_ENU, 0, 0, yaw_in_deg];

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
    plot3(SensorMount_center(1,1), SensorMount_center(1,2), SensorMount_center(1,3), 'b.', 'MarkerSize', 50);
    
    % Plot the Vehicle origin 
    plot3(vehicleOrigin_ENU(1,1), vehicleOrigin_ENU(1,2), vehicleOrigin_ENU(1,3), 'g.', 'MarkerSize', 50);
end

if flag_do_debug
    fprintf(fileID,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end